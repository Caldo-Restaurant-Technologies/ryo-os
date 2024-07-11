use crate::bag_handler::BagHandler;
use crate::config::*;
use crate::ryo::{make_hatches, RyoIo};
use control_components::components::clear_core_motor::{ClearCoreMotor, Status};
use control_components::components::scale::{Scale, ScaleCmd};
use control_components::controllers::{clear_core, ek1100_io};
use env_logger::Env;
use log::info;
use std::time::Duration;
use std::{array, env};
use futures::future::join_all;
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tokio::task::JoinSet;

pub mod config;

pub mod hmi;
pub mod recipe_handling;

pub mod bag_handler;
pub mod manual_control;
pub mod ryo;

type CCController = clear_core::Controller;
type EtherCATIO = ek1100_io::Controller;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();
    let host = env::args()
        .nth(1)
        .expect("Is this running locally or on Ryo?");

    //TODO: Change so that interface can be defined as a compiler flag passed at compile time
    // Figure out a way to detect at launch

    let interface = || match host.as_str() {
        "local-test" => LOCAL_INTERFACE,
        "ryo" => RYO_INTERFACE,
        _ => RYO_INTERFACE,
    };

    let mut client_set = JoinSet::new();

    //Create IO controllers and their relevant clients
    let (cc1, cl1) = CCController::with_client(CLEAR_CORE_1_ADDR, CC1_MOTORS.as_slice());
    let (cc2, cl2) = CCController::with_client(CLEAR_CORE_2_ADDR, CC2_MOTORS.as_slice());
    let (etc_io, cl3) = EtherCATIO::with_client(interface(), 2);

    client_set.spawn(cl1);
    client_set.spawn(cl2);
    client_set.spawn(cl3);

    let scale_txs: [Sender<ScaleCmd>; 4] = array::from_fn(|i| {
        let phidget_id = PHIDGET_SNS[i];
        let (tx, actor) = Scale::actor_tx_pair(phidget_id);
        client_set.spawn(actor);
        info!("Spawned {phidget_id} client-actor");
        tx
    });

    info!("Controller-Client pairs created successfully");

    let ryo_io = RyoIo {
        cc1,
        cc2,
        etc_io,
        scale_txs,
    };

    let (_, cycle_rx) = channel::<CycleCmd>(10);

    cycle(ryo_io, cycle_rx).await;

    while let Some(_) = client_set.join_next().await {}
    Ok(())
}

pub enum CycleCmd {
    Cycle(usize),
    Pause,
    Cancel,
}

async fn pull_before_flight(io: RyoIo) {
    let cc1_motors: [ClearCoreMotor; 3] = array::from_fn(|motor_id| io.cc1.get_motor(motor_id));
    let cc2_motors: [ClearCoreMotor; 4] = array::from_fn(|motor_id| io.cc2.get_motor(motor_id));

    let enable_cc1_handles = cc1_motors.iter().map(|motor|{
        async move {
            motor.enable().await.unwrap();
        }
    });
    let enable_cc2_handles = cc2_motors.iter().map(|motor|{
        async move {
            motor.enable().await.unwrap();
        }
    });
    join_all(enable_cc1_handles).await;
    join_all(enable_cc2_handles).await;

    let cc1_motors: [ClearCoreMotor; 3] = array::from_fn(|motor_id| io.cc1.get_motor(motor_id));
    let cc2_motors: [ClearCoreMotor; 4] = array::from_fn(|motor_id| io.cc2.get_motor(motor_id));

    let clear_cc1_handles = cc1_motors.iter().map(|motor|{
        async move {
            motor.clear_alerts().await;
        }
    });
    let clear_cc2_handles = cc2_motors.iter().map(|motor|{
        async move {
            motor.clear_alerts().await;
        }
    });
    join_all(clear_cc1_handles).await;
    join_all(clear_cc2_handles).await;
    
    let mut set = JoinSet::new();
    let hatches = make_hatches(io.cc1.clone(), io.cc2.clone());
    let bag_handler = BagHandler::new(io.cc1.clone(), io.cc2.clone());
    let gantry = io.cc1.get_motor(GANTRY_MOTOR_ID);

    for mut hatch in hatches {
        set.spawn(async move { hatch.timed_close(Duration::from_secs_f64(2.8)).await });
    }

    set.spawn(async move { bag_handler.dispense_bag().await });
    set.spawn(async move {
        gantry.enable().await.expect("Motor is faulted");
        let state = gantry.get_status().await;
        if state == Status::Moving {
            gantry.wait_for_move(Duration::from_secs(1)).await;
        }
        // gantry.absolute_move(-0.25).await.expect("Motor is faulted");
        // gantry.wait_for_move(Duration::from_secs(1)).await;
    });
    info!("All systems go.");
    while let Some(_) = set.join_next().await {}
}

async fn cycle(io: RyoIo, mut auto_rx: Receiver<CycleCmd>) {
    // Create drive channels and spawn clients

    pull_before_flight(io.clone()).await;

    let mut batch_count = 0;
    let mut pause = false;
    loop {
        match auto_rx.try_recv() {
            Ok(msg) => match msg {
                CycleCmd::Cycle(count) => {
                    batch_count = count;
                }
                CycleCmd::Pause => {
                    pause = true;
                }
                CycleCmd::Cancel => {
                    batch_count = 0;
                }
            },
            _ => {}
        }

        if batch_count > 0 {
            while pause {
                tokio::time::sleep(Duration::from_secs(2)).await;
                info!("System Paused.");
            }
        }
    }
}

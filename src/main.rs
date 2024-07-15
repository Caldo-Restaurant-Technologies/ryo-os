use crate::bag_handler::BagHandler;
use crate::config::*;
use crate::manual_control::enable_and_clear_all;
use crate::ryo::{make_dispensers, make_gantry, make_gripper, make_hatches, make_sealer, make_trap_door, RyoIo};
use control_components::components::clear_core_motor::{ClearCoreMotor, Status};
use control_components::components::scale::{Scale, ScaleCmd};
use control_components::controllers::{clear_core, ek1100_io};
use control_components::subsystems::dispenser::{Parameters, Setpoint};
use control_components::subsystems::sealer::Sealer;
use env_logger::Env;
use futures::future::join_all;
use log::{error, info};
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use std::{array, env};
use std::net::SocketAddr;
use control_components::components::clear_core_io::HBridgeState;
use tokio::join;
use tokio::sync::mpsc;
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tokio::task::{spawn_blocking, JoinHandle, JoinSet};
use tokio::time::{sleep, sleep_until};

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

    let task = env::args()
        .nth(2)
        .expect("Do you want to run a cycle or hmi?");

    //TODO: Change so that interface can be defined as a compiler flag passed at compile time
    // Figure out a way to detect at launch

    let interface = || match host.as_str() {
        "local-test" => LOCAL_INTERFACE,
        "ryo" => RYO_INTERFACE,
        _ => RYO_INTERFACE,
    };

    let mut client_set = JoinSet::new();

    let scales_handles: [JoinHandle<Scale>; 4] = array::from_fn(|scale_id| {
        spawn_blocking(move || {
            let scale = Scale::new(PHIDGET_SNS[scale_id]);
            scale.connect().unwrap()
        })
    });
    let mut scales = join_all(scales_handles).await;
    scales.reverse();
    let scale_txs: [Sender<ScaleCmd>; 4] = array::from_fn(|_scale_id| {
        let (tx, actor) = scales.pop().unwrap().unwrap().actor_tx_pair();
        client_set.spawn(actor);
        // info!("Spawned {phidget_id} client-actor");
        tx
    });

    //Create IO controllers and their relevant clients
    let (cc1, cl1) = CCController::with_client(CLEAR_CORE_1_ADDR, CC1_MOTORS.as_slice());
    let (cc2, cl2) = CCController::with_client(CLEAR_CORE_2_ADDR, CC2_MOTORS.as_slice());
    let (etc_io, cl3) = EtherCATIO::with_client(interface(), 2);

    client_set.spawn(cl3);
    sleep(Duration::from_secs(2)).await;
    client_set.spawn(cl1);
    client_set.spawn(cl2);

    info!("Controller-Client pairs created successfully");

    let ryo_io = RyoIo {
        cc1,
        cc2,
        etc_io,
        scale_txs,
    };

    let (_, cycle_rx) = channel::<CycleCmd>(10);

    match task.as_str() {
        "hmi" => hmi(ryo_io, cycle_rx).await,
        "cycle" => cycle(ryo_io, cycle_rx).await,
        _ => {
            error!("Must enter hmi or cycle");
            return Ok(())
        }
    }

    while let Some(_) = client_set.join_next().await {}
    Ok(())
}

pub enum CycleCmd {
    Cycle(usize),
    Pause,
    Cancel,
}

async fn pull_before_flight(io: RyoIo) {
    enable_and_clear_all(io.clone()).await;
    sleep(Duration::from_millis(500)).await;

    let mut set = JoinSet::new();
    let hatches = make_hatches(io.cc1.clone(), io.cc2.clone());
    let bag_handler = BagHandler::new(io.cc1.clone(), io.cc2.clone());
    let gantry = make_gantry(io.cc1.clone());
    gantry.set_velocity(30.).await;
    // make_trap_door(io.clone()).actuate(HBridgeState::Pos).await;
    make_gripper(io.cc1.clone(), io.cc2.clone()).close().await;
    make_sealer(io.clone()).seal().await;


    make_trap_door(io.clone()).actuate(HBridgeState::Pos).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    make_trap_door(io.clone()).actuate(HBridgeState::Off).await;
  

    for mut hatch in hatches {
        set.spawn(
            async move { 
                hatch.timed_open(HATCHES_OPEN_TIME).await;
                hatch.timed_close(Duration::from_secs_f64(1.6)).await;
            }
        );
    }

    set.spawn(async move { bag_handler.dispense_bag().await });
    set.spawn(async move {
        gantry.enable().await.expect("Motor is faulted");
        let state = gantry.get_status().await;
        if state == Status::Moving {
            gantry.wait_for_move(Duration::from_secs(1)).await;
        }
        let _ = gantry
            .absolute_move(GANTRY_HOME_POSITION)
            .await;
        gantry.wait_for_move(Duration::from_secs(1)).await;
    });

    drop(io);
    info!("All systems go.");
    while let Some(_) = set.join_next().await {}
}

async fn cycle(io: RyoIo, mut auto_rx: Receiver<CycleCmd>) {

    let shutdown = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");



    let mut batch_count = 0;
    let mut pause = false;
    loop {
        info!("Cycle loop");
        if shutdown.load(Ordering::Relaxed) {
            break;
        }

        pull_before_flight(io.clone()).await;

        // Create Dispense Tasks
        let params: [Parameters; 4] = array::from_fn(|_| Parameters::default());
        let set_points: [Setpoint; 4] =
            array::from_fn(|_| Setpoint::Timed(Duration::from_secs(15)));
        let dispensers = make_dispensers(io.cc2.clone(), &set_points, &params, &io.scale_txs);
        let dispense_tasks: Vec<JoinHandle<()>> = dispensers
            .into_iter()
            .map(|dispenser| {
                tokio::spawn(async move { dispenser.dispense(DISPENSER_TIMEOUT).await })
            })
            .collect();

        // Create Bag Loading Task
        let mut bag_handler = BagHandler::new(io.cc1.clone(), io.cc2.clone());
        let bag_load_task = tokio::spawn(async move { bag_handler.load_bag().await });

        // Concurrently run Dispensing and Bag Loading
        let _ = join!(join_all(dispense_tasks), bag_load_task);

        // Fill Bag
        let gantry = make_gantry(io.cc1.clone());
        let mut hatches = make_hatches(io.cc1.clone(), io.cc2.clone());
        hatches.reverse();
        for id in 0..4 {
            info!("Going to Node {:}", id);
            let _ = gantry
                .absolute_move(GANTRY_NODE_POSITIONS[id])
                .await;
            gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
            let mut hatch = hatches.pop().unwrap();
            hatch.timed_open(HATCHES_OPEN_TIME).await;
            sleep(Duration::from_millis(500)).await;
            hatch.timed_close(HATCH_CLOSE_TIMES[id]).await;
        }

        // Drop Bag
        let _ = gantry
            .absolute_move(GANTRY_BAG_DROP_POSITION)
            .await;
        gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
        let mut gripper = make_gripper(io.cc1.clone(), io.cc2.clone());
        gripper.open().await;
        sleep(Duration::from_millis(500)).await;
        gripper.close().await;

        // Seal Bag
        make_sealer(io.clone()).seal().await;

        // Release Bag
        let mut trap_door =make_trap_door(io.clone());
        trap_door.actuate(HBridgeState::Neg).await;
        sleep(SEALER_MOVE_DOOR_TIME).await;
        trap_door.actuate(HBridgeState::Off).await;
        sleep(Duration::from_millis(500)).await;
        trap_door.actuate(HBridgeState::Pos).await;
        sleep(SEALER_MOVE_DOOR_TIME).await;
        trap_door.actuate(HBridgeState::Off).await;

        sleep(Duration::from_secs(10)).await;

        // Re-home gantry and dispense new bag
        let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
        BagHandler::new(io.cc1.clone(), io.cc2.clone()).dispense_bag().await;
        gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;

        sleep(Duration::from_secs(5)).await;

        // match auto_rx.try_recv() {
        //     Ok(msg) => match msg {
        //         CycleCmd::Cycle(count) => {
        //             batch_count = count;
        //         }
        //         CycleCmd::Pause => {
        //             pause = true;
        //         }
        //         CycleCmd::Cancel => {
        //             batch_count = 0;
        //         }
        //     },
        //     _ => {}
        // }
        // sleep(Duration::from_secs(1)).await;
        // if batch_count > 0 {
        //
        // }




    }
}

async fn hmi(io: RyoIo, mut auto_rx: Receiver<CycleCmd>) {

    let shutdown = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");
    info!("HMI Ready");

    let state_server = tokio::spawn(
        hmi::ui_server(
            SocketAddr::from(([0, 0, 0, 0], 3000)),
            io.clone(),
            shutdown.clone(),
        )
    );
    let _ = join!(state_server);
    
    // let mut batch_count = 0;
    // let mut pause = false;
    // loop {
    //     info!("Cycle loop");
    //     if shutdown.load(Ordering::Relaxed) {
    //         break;
    //     }
    // 
    //     // let (hmi_tx, mut hmi_rx) = channel(1);
    //     let state_server = tokio::spawn(
    //         hmi::ui_server(
    //             SocketAddr::from(([0, 0, 0, 0], 3000)),
    //             io.clone(),
    //             shutdown.clone(),
    //         )
    //     );
    //     let _ = join!(state_server);
    // 
    //     // match auto_rx.try_recv() {
    //     //     Ok(msg) => match msg {
    //     //         CycleCmd::Cycle(count) => {
    //     //             batch_count = count;
    //     //         }
    //     //         CycleCmd::Pause => {
    //     //             pause = true;
    //     //         }
    //     //         CycleCmd::Cancel => {
    //     //             batch_count = 0;
    //     //         }
    //     //     },
    //     //     _ => {}
    //     // }
    //     // sleep(Duration::from_secs(1)).await;
    //     // if batch_count > 0 {
    //     //     loop {
    //     //         pull_before_flight(io.clone()).await;
    //     // 
    //     //         // Create Dispense Tasks
    //     //         let params: [Parameters; 4] = array::from_fn(|_| Parameters::default());
    //     //         let set_points: [Setpoint; 4] =
    //     //             array::from_fn(|_| Setpoint::Timed(Duration::from_secs(15)));
    //     //         let dispensers = make_dispensers(io.cc2.clone(), &set_points, &params, &io.scale_txs);
    //     //         let dispense_tasks: Vec<JoinHandle<()>> = dispensers
    //     //             .into_iter()
    //     //             .map(|dispenser| {
    //     //                 tokio::spawn(async move { dispenser.dispense(DISPENSER_TIMEOUT).await })
    //     //             })
    //     //             .collect();
    //     // 
    //     //         // Create Bag Loading Task
    //     //         let mut bag_handler = BagHandler::new(io.cc1.clone(), io.cc2.clone());
    //     //         let bag_load_task = tokio::spawn(async move { bag_handler.load_bag().await });
    //     // 
    //     //         // Concurrently run Dispensing and Bag Loading
    //     //         let _ = join!(join_all(dispense_tasks), bag_load_task);
    //     // 
    //     //         // Fill Bag
    //     //         let gantry = make_gantry(io.cc1.clone());
    //     //         let mut hatches = make_hatches(io.cc1.clone(), io.cc2.clone());
    //     //         hatches.reverse();
    //     //         for id in 0..4 {
    //     //             info!("Going to Node {:}", id);
    //     //             let _ = gantry
    //     //                 .absolute_move(GANTRY_NODE_POSITIONS[id])
    //     //                 .await;
    //     //             gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
    //     //             let mut hatch = hatches.pop().unwrap();
    //     //             hatch.timed_open(HATCHES_OPEN_TIME).await;
    //     //             sleep(Duration::from_millis(500)).await;
    //     //             hatch.timed_close(HATCH_CLOSE_TIMES[id]).await;
    //     //         }
    //     // 
    //     //         // Drop Bag
    //     //         let _ = gantry
    //     //             .absolute_move(GANTRY_BAG_DROP_POSITION)
    //     //             .await;
    //     //         gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
    //     //         let mut gripper = make_gripper(io.cc1.clone(), io.cc2.clone());
    //     //         gripper.open().await;
    //     //         sleep(Duration::from_millis(500)).await;
    //     //         gripper.close().await;
    //     // 
    //     //         // Seal Bag
    //     //         make_sealer(io.clone()).seal().await;
    //     // 
    //     //         // Release Bag
    //     //         let mut trap_door = make_trap_door(io.clone());
    //     //         trap_door.actuate(HBridgeState::Neg).await;
    //     //         sleep(SEALER_MOVE_DOOR_TIME).await;
    //     //         trap_door.actuate(HBridgeState::Off).await;
    //     //         sleep(Duration::from_millis(500)).await;
    //     //         trap_door.actuate(HBridgeState::Pos).await;
    //     //         sleep(SEALER_MOVE_DOOR_TIME).await;
    //     //         trap_door.actuate(HBridgeState::Off).await;
    //     // 
    //     //         sleep(Duration::from_secs(10)).await;
    //     // 
    //     //         // Re-home gantry and dispense new bag
    //     //         let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
    //     //         BagHandler::new(io.cc1.clone(), io.cc2.clone()).dispense_bag().await;
    //     //         gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
    //     // 
    //     //         sleep(Duration::from_secs(5)).await;
    //     //     }
    //     // }
    // 
    // 
    // 
    // 
    // }
}
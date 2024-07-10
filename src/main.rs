use crate::config::*;
use log::info;
use std::{array, env};
use tokio::sync::mpsc::{channel, Receiver, Sender};
use control_components::controllers::{clear_core, ek1100_io};
use env_logger::Env;
use std::time::Duration;
use control_components::components::clear_core_motor::Status;
use control_components::components::scale::{Scale, ScaleCmd};
use tokio::task::JoinSet;
use crate::bag_handler::BagHandler;
use crate::ryo::make_hatches;


pub mod config;

pub mod hmi;
pub mod recipe_handling;

pub mod bag_handler;
mod ryo;

type CCController = clear_core::Controller;
type EtherCATIO = ek1100_io::Controller;

#[derive(Clone)]
pub struct RyoIo{
    pub cc1: CCController, 
    pub cc2: CCController, 
    pub etc_io: EtherCATIO, 
    pub scale_txs: [Sender<ScaleCmd>; 4]
}



#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();
    let host = env::args()
        .nth(1)
        .expect("Is this running locally or on Ryo?");

    //TODO: Change so that interface can be defined as a compiler flag passed at compile time
    // Figure out a way to detect at launch

    let interface = ||{
        match host.as_str() {
            "local-test" => {LOCAL_INTERFACE},
            "ryo" => {RYO_INTERFACE},
            _ => {RYO_INTERFACE}
        }
    };
    
    let mut client_set = JoinSet::new();
    

    //Create IO controllers and their relevant clients
    let (cc1, cl1) = CCController::with_client(CLEAR_CORE_1_ADDR, CC1_MOTORS.as_slice());
    let (cc2, cl2) = CCController::with_client(CLEAR_CORE_2_ADDR, CC2_MOTORS.as_slice());
    let (etc_io, cl3) = EtherCATIO::with_client(interface(), 2);

    client_set.spawn(cl1);
    client_set.spawn(cl2);
    client_set.spawn(cl3);
   
    let scale_txs : [Sender<ScaleCmd>; 4] = array::from_fn(|i|{
        let (tx, actor) = Scale::actor_tx_pair(PHIDGET_SNS[i]);
        client_set.spawn(actor);
        tx
    });

   
    info!("Controller-Client pairs created successfully");



    let ryo_io = RyoIo{cc1, cc2, etc_io, scale_txs};

    let (_, cycle_rx) = channel::<CycleCmd>(10);

    cycle(ryo_io, cycle_rx).await;

    while let Some(_) = client_set.join_next().await {}
    Ok(())
}



pub enum CycleCmd{
    Cycle(usize),
    Pause,
    Cancel
}


async fn pull_before_flight(io: RyoIo) {
    let mut set = JoinSet::new();
    let hatches = make_hatches(io.cc1.clone(), io.cc2.clone());
    let bag_handler = BagHandler::new(io.cc1.clone(), io.cc2.clone());
    let gantry = io.cc1.get_motor(GANTRY_MOTOR_ID);
    
    for mut hatch in hatches {
        set.spawn(async move { hatch.timed_close(Duration::from_secs_f64(2.8)).await});
    }
    
    set.spawn(async move{bag_handler.dispense_bag().await});
    set.spawn(async move{ 
        gantry.enable().await.expect("Motor is faulted");
        let state = gantry.get_status().await;
        if state == Status::Moving {
            gantry.wait_for_move(Duration::from_secs(1)).await;
        }
        gantry.enable().await.expect("Motor is faulted");
        gantry.absolute_move(-0.25).await.expect("Motor is faulted");
        gantry.wait_for_move(Duration::from_secs(1)).await;
    });
    info!("All systems go.");
    while let Some(_) = set.join_next().await {}
}



async fn cycle(io: RyoIo, mut auto_rx: Receiver<CycleCmd>)  {
    // Create drive channels and spawn clients


    pull_before_flight(io.clone()).await;


    let mut batch_count = 0;
    let mut pause = false;
    loop {

        match auto_rx.try_recv() {
            Ok(msg) => {
                match msg {
                    CycleCmd::Cycle(count) => {
                        batch_count = count;
                    }
                    CycleCmd::Pause => {
                        pause = true;
                    }
                    CycleCmd::Cancel => {
                        batch_count = 0;
                    }
                }
            }
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


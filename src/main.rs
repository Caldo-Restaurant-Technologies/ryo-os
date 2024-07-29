use crate::app_integration::RyoFirebaseClient;
use crate::bag_handler::BagHandler;
use crate::config::*;
use crate::hmi::ui_server_with_fb;
use crate::ryo::{
    drop_bag, dump_from_hatch, make_bag_handler, make_bag_load_task,
    make_bag_sensor,
    make_dispense_tasks, make_gantry, make_sealer, pull_after_flight,
    pull_before_flight, release_bag_from_sealer, BagFilledState, BagState, NodeState,
    RyoFailure, RyoIo, RyoRunState, RyoState,
};
use control_components::components::clear_core_motor::Status;
use control_components::components::scale::{Scale, ScaleCmd};
use control_components::controllers::{clear_core, ek1100_io};
use control_components::subsystems::bag_handling::BagSensorState;
use env_logger::Env;
use futures::future::join_all;
use log::{error, info, warn};
use std::net::SocketAddr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use std::{array, env};
use tokio::sync::mpsc::{Sender};
use tokio::sync::Mutex;
use tokio::task::{spawn_blocking, JoinHandle, JoinSet};
use tokio::time::sleep;
use crate::ryo::RyoRunState::Faulted;

pub mod config;

pub mod app_integration;
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
    
    let run_state = match task.as_str() {
        "cycle" => RyoRunState::Ready,
        "hmi" => RyoRunState::UI,
        _ => RyoRunState::NewJob,
    };

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
            let mut scale = Scale::new(PHIDGET_SNS[scale_id]);
            scale = Scale::change_coefficients(scale, NODE_COEFFICIENTS[scale_id].to_vec());
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
    let (etc_io, cl3) = EtherCATIO::with_client(interface(), ETHERCAT_NUMBER_OF_SLOTS);

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

    let gantry = make_gantry(ryo_io.cc1.clone()).await;
    gantry.clear_alerts().await;
    let _ = gantry.enable().await;
    sleep(Duration::from_secs(10)).await;
    info!("Connecting to Firebase");
    let mut firebase = RyoFirebaseClient::new();
    let app_state = Arc::new(Mutex::new(app_integration::Status::default()));
    // let job_order = Arc::new(Mutex::new(app_integration::JobOrder::default()));
    // let system_mode = Arc::new(Mutex::new(app_integration::SystemMode::default()));
    let mut state;
    let shutdown = Arc::new(AtomicBool::new(false));

    let app_state_for_fb = app_state.clone();
    // let job_order_for_fb = job_order.clone();
    // let system_mode_for_fb = system_mode.clone();
    let shutdown_app = shutdown.clone();
    let app_scales = ryo_io.scale_txs.clone();
    let app_handler = tokio::spawn(async move {
        firebase
            .update(app_scales.as_slice(), app_state_for_fb, shutdown_app)
            .await;
    });

    loop {
        state = gantry.get_status().await;
        match state {
            Status::Ready => break,
            Status::Moving => continue,
            Status::Faulted => gantry.clear_alerts().await,
            Status::Disabled => {
                let _ = gantry.enable().await;
            }
            Status::Enabling => continue,
            Status::Unknown => {
                error!("Gantry in unknown state");
            }
        }
        info!("Current Gantry State: {:?}", state);
        sleep(Duration::from_secs(3)).await;
    }

    info!("Gantry status: {:?}", gantry.get_status().await);
    gantry.set_acceleration(GANTRY_ACCELERATION).await;
    gantry.set_deceleration(GANTRY_ACCELERATION).await;
    gantry.set_velocity(GANTRY_VELOCITY).await;
    let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
    gantry.wait_for_move(Duration::from_secs(1)).await.unwrap();

    info!("Starting cycle loop");
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");
    let mut ryo_state = RyoState::new();
    ryo_state.set_run_state(run_state);
    loop {
        if shutdown.load(Ordering::Relaxed) {
            break;
        }
        ryo_state.check_failures();
        match ryo_state.get_run_state() {
            RyoRunState::NewJob => {
                info!("Starting cycle");
                ryo_state = pull_before_flight(ryo_io.clone()).await;
                // ryo_state = single_cycle(ryo_state, ryo_io.clone()).await;
            }
            RyoRunState::Running => {
                ryo_state = single_cycle(ryo_state, ryo_io.clone()).await;
            }
            RyoRunState::UI => {
                hmi_with_fb(ryo_io.clone(), ryo_state.clone()).await;
            }
            RyoRunState::Ready | RyoRunState::Faulted => {
                ryo_state = app_state.lock().await.update_ryo_state(ryo_state).await;
            }
        }
    }

    let _ = app_handler.await;
    while (client_set.join_next().await).is_some() {}
    Ok(())
}

pub enum CycleCmd {
    Cycle(usize),
    Pause,
    Cancel,
} 

async fn single_cycle(mut state: RyoState, io: RyoIo) -> RyoState {
    state.update_node_levels(io.clone()).await;
    info!("Ryo State: {:?}", state);

    let mut dispense_and_bag_tasks = Vec::new();
    match state.get_bag_state() {
        BagState::Bagful(BagFilledState::Filled) => {
            info!("Bag already filled")
        }
        BagState::Bagful(BagFilledState::Filling) | BagState::Bagful(BagFilledState::Empty) | BagState::Bagless => {
            info!("Bag not full, dispensing");
            (state, dispense_and_bag_tasks) = make_dispense_tasks(state.clone(), io.clone());
        }
    }

    match state.get_bag_state() {
        BagState::Bagless => {
            info!("Getting bag");
            make_bag_handler(io.clone()).dispense_bag().await;
            let gantry = make_gantry(io.cc1.clone()).await;
            let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
            gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
            dispense_and_bag_tasks.push(make_bag_load_task(io.clone()));
            // TODO: maybe have above return results so we know whether to update states?
            state.set_bag_state(BagState::Bagful(BagFilledState::Filling));
        }
        BagState::Bagful(_) => {
            info!("Bag already loaded");
        }
    }

    let _ = join_all(dispense_and_bag_tasks).await;
    

    match state.get_bag_state() {
        BagState::Bagful(BagFilledState::Empty) | BagState::Bagful(BagFilledState::Filling) => {
            info!("Bag not filled, dumping from hatches");
            state.set_bag_state(BagState::Bagful(BagFilledState::Filling));
            let bag_sensor = make_bag_sensor(io.clone());
            let gantry = make_gantry(io.cc1.clone()).await;
            for node in 0..NUMBER_OF_NODES {
                match state.get_node_state(node) {
                    NodeState::Dispensed => {
                        let _ = gantry.absolute_move(GANTRY_NODE_POSITIONS[node]).await;
                        gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
                        match bag_sensor.check().await {
                            BagSensorState::Bagful => {
                                info!("Dispensing from Node {:?}", node);
                                dump_from_hatch(node, io.clone()).await;
                                state.set_node_state(node, NodeState::Ready);
                                state.set_bag_state(BagState::Bagful(BagFilledState::Filled));
                            }
                            BagSensorState::Bagless => {
                                state.set_bag_state(BagState::Bagless);
                                error!("Lost bag");
                                state.log_failure(RyoFailure::BagDispenseFailure);
                                return state;
                            }
                        }
                    }
                    NodeState::Ready => (),
                    NodeState::Empty => {
                        // TODO: hmmm figure out how to reimplement this but with single node dispensing (or not needed?)
                        // error!("Node {:?} is empty", node);
                        // state.set_run_state(Faulted);
                        // return state;
                    }
                }
            }
        }
        BagState::Bagful(BagFilledState::Filled) => {
            info!("Bag already filled");
        }
        BagState::Bagless => {
            warn!("No bag, retrying");
            return state;
        }
    }

    let io_clone = io.clone();
    tokio::spawn(async move {
        BagHandler::new(io_clone).dispense_bag().await;
        info!("New bag dispensed");
    });

    drop_bag(io.clone()).await;

    match make_bag_sensor(io.clone()).check().await {
        BagSensorState::Bagless => {
            state.set_bag_state(BagState::Bagless);
        }
        BagSensorState::Bagful => {
            error!("Failed to drop bag");
            state.log_failure(RyoFailure::BagDroppingFailure);
            return state;
        }
    }
    
    let _ = make_gantry(io.cc1.clone()).await.absolute_move(GANTRY_HOME_POSITION).await;

    let io_clone = io.clone();
    tokio::spawn(async move {
        make_sealer(io_clone.clone())
            .timed_move_seal(SEALER_MOVE_TIME)
            .await;
        release_bag_from_sealer(io_clone.clone()).await;
    });

    // pull_after_flight(io).await;

    state.clear_failures();
    // TODO: prob put it back in ready and up a counter of cycles run? will work on with firebase integration
    state.set_run_state(RyoRunState::NewJob);
    state
}

async fn hmi_with_fb(io: RyoIo, ryo_state: RyoState) {
    let shutdown = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");
    info!("HMI Ready");

    ui_server_with_fb(
        SocketAddr::from(([0, 0, 0, 0], 3000)),
        io.clone(),
        ryo_state,
        shutdown.clone(),
    )
    .await
    .unwrap();
    drop(io);
}

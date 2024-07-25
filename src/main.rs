use crate::app_integration::RyoFirebaseClient;
use crate::bag_handler::BagHandler;
use crate::config::*;
use crate::manual_control::enable_and_clear_all;
use crate::ryo::RyoRunState::Ready;
use crate::ryo::{
    drop_bag, dump_from_hatch, make_and_close_hatch, make_bag_handler, make_bag_load_task,
    make_bag_sensor, make_default_dispense_tasks, make_default_weighed_dispense_tasks, make_gantry,
    make_hatch, make_sealer, make_trap_door, pull_after_flight, release_bag_from_sealer,
    BagFilledState, BagLoadedState, NodeState, RyoFailure, RyoIo, RyoRunState, RyoState,
};
use control_components::components::clear_core_io::HBridgeState;
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
use tokio::sync::mpsc::{channel, Receiver, Sender};
use tokio::sync::Mutex;
use tokio::task::{spawn_blocking, JoinHandle, JoinSet};
use tokio::time::sleep;

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

    let n_nodes = match env::args().nth(3) {
        Some(n) => match n.as_str() {
            "1" => 1,
            "2" => 2,
            "3" => 3,
            _ => 4,
        },
        None => 4,
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
    let mut firebase = RyoFirebaseClient::new();
    let app_state = Arc::new(Mutex::new(app_integration::Status::default()));
    let mut state;
    let shutdown = Arc::new(AtomicBool::new(false));
    let app_state_for_fb = app_state.clone();
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

    match task.as_str() {
        "hmi" => hmi(ryo_io).await,
        "cycle" => {
            signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
                .expect("Register hook");

            pull_before_flight(ryo_io.clone()).await;
            let mut ryo_state = RyoState::fresh();
            loop {
                if shutdown.load(Ordering::Relaxed) {
                    break;
                }
                ryo_state = app_state.lock().await.update_ryo_state(ryo_state);

                match ryo_state.get_run_state() {
                    RyoRunState::Running => {
                        ryo_state.check_failures();
                        info!("Cycling");
                        ryo_state = single_cycle(ryo_state, ryo_io.clone()).await;
                    }
                    // TODO: figure out how to differentiate these
                    RyoRunState::Ready | RyoRunState::Faulted => (),
                }
            }
        }
        _ => {
            error!("Must enter hmi or cycle");
            return Ok(());
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

async fn pull_before_flight(io: RyoIo) {
    enable_and_clear_all(io.clone()).await;
    let gantry = make_gantry(io.cc1.clone()).await;
    loop {
        sleep(Duration::from_secs(1)).await;
        if gantry.get_status().await == Status::Ready {
            break;
        }
    }
    for node in 0..4 {
        let motor = io.cc2.get_motor(node);
        motor.set_velocity(0.5).await;
        motor.set_acceleration(90.).await;
        motor.set_deceleration(90.).await;
    }

    // set_motor_accelerations(io.clone(), 50.).await;
    sleep(Duration::from_millis(500)).await;

    let mut set = JoinSet::new();
    let bag_handler = BagHandler::new(io.clone());

    // make_trap_door(io.clone()).actuate(HBridgeState::Pos).await;
    make_bag_handler(io.clone()).close_gripper().await;
    make_sealer(io.clone())
        .absolute_move(SEALER_RETRACT_SET_POINT)
        .await;
    info!("Sealer retracted");

    make_trap_door(io.clone()).actuate(HBridgeState::Pos).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    make_trap_door(io.clone()).actuate(HBridgeState::Off).await;
    info!("Trap door opened");

    for id in 0..4 {
        let io_clone = io.clone();
        info!("Closing Hatch {:?}", id);
        // make_and_close_hatch(id, io_clone).await;
        set.spawn(async move {
            info!("Closing Hatch {:?}", id);
            make_and_close_hatch(id, io_clone).await;
        });
    }

    set.spawn(async move { bag_handler.dispense_bag().await });
    set.spawn(async move {
        gantry.enable().await.expect("Motor is faulted");
        let state = gantry.get_status().await;
        if state == Status::Moving {
            gantry.wait_for_move(Duration::from_secs(1)).await.unwrap();
        }
        let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
        gantry.wait_for_move(Duration::from_secs(1)).await.unwrap();
    });

    drop(io);
    info!("All systems go.");
    while (set.join_next().await).is_some() {}
}

async fn single_cycle(mut state: RyoState, io: RyoIo) -> RyoState {
    info!("Ryo State: {:?}", state);

    let mut node_ids = Vec::with_capacity(NUMBER_OF_NODES);
    match state.get_bag_filled_state() {
        Some(BagFilledState::Filled) => {
            info!("Bag already filled");
        }
        Some(BagFilledState::Filling) | Some(BagFilledState::Empty) | None => {
            info!("Bag not full, dispensing");
            for id in 0..NUMBER_OF_NODES {
                if state.get_dispenser_recipe(id).is_some() {
                    match state.get_node_state(id) {
                        NodeState::Ready => {
                            node_ids.push(id);
                        }
                        NodeState::Dispensed => (),
                    }
                }
            }
        }
    }
    // let mut dispense_and_bag_tasks = make_default_dispense_tasks(node_ids, io.clone());
    let mut dispense_and_bag_tasks =
        make_default_weighed_dispense_tasks(225., node_ids, io.clone());

    match state.get_bag_loaded_state() {
        BagLoadedState::Bagless => {
            info!("Getting bag");
            make_bag_handler(io.clone()).dispense_bag().await;
            let gantry = make_gantry(io.cc1.clone()).await;
            let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
            gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
            dispense_and_bag_tasks.push(make_bag_load_task(io.clone()));
            let _ = join_all(dispense_and_bag_tasks).await;
            // TODO: maybe have above return results so we know whether to update states?
            state.set_bag_loaded_state(BagLoadedState::Bagful);
            state.set_all_node_states(NodeState::Dispensed);
            state.set_bag_filled_state(Some(BagFilledState::Filling));
        }
        BagLoadedState::Bagful => {
            info!("Bag already loaded");
        }
    }

    let io_clone = io.clone();
    tokio::spawn(async move {
        BagHandler::new(io_clone).dispense_bag().await;
    });

    match state.get_bag_filled_state() {
        Some(BagFilledState::Empty) | Some(BagFilledState::Filling) => {
            info!("Bag not filled, dumping from hatches");
            state.set_bag_filled_state(Some(BagFilledState::Filling));
            let bag_sensor = make_bag_sensor(io.clone());
            let gantry = make_gantry(io.cc1.clone()).await;
            for node in 0..NUMBER_OF_NODES {
                if state.get_dispenser_recipe(node).is_some() {
                    let _ = gantry.absolute_move(GANTRY_NODE_POSITIONS[node]).await;
                    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
                    match bag_sensor.check().await {
                        BagSensorState::Bagful => {
                            match state.get_node_state(node) {
                                NodeState::Dispensed => {
                                    info!("Dispensing from Node {:?}", node);
                                    dump_from_hatch(node, io.clone()).await;
                                    state.set_node_state(node, NodeState::Ready);
                                }
                                NodeState::Ready => (), // TODO:: this is an unreachable case?
                            }
                        }
                        BagSensorState::Bagless => {
                            state.set_bag_loaded_state(BagLoadedState::Bagless);
                            error!("Lost bag");
                            state.log_failure(RyoFailure::BagDispenseFailure);
                            return state;
                        }
                    }
                }
            }
            state.set_bag_filled_state(Some(BagFilledState::Filled));
        }
        Some(BagFilledState::Filled) => {
            info!("Bag already filled");
        }
        None => {
            warn!("Bag not filled, retrying");
            return state;
        }
    }

    drop_bag(io.clone()).await;

    match make_bag_sensor(io.clone()).check().await {
        BagSensorState::Bagless => {
            state.set_bag_loaded_state(BagLoadedState::Bagless);
            state.set_bag_filled_state(None);
        }
        BagSensorState::Bagful => {
            error!("Failed to drop bag");
            state.log_failure(RyoFailure::BagDroppingFailure);
            return state;
        }
    }

    let io_clone = io.clone();
    tokio::spawn(async move {
        make_sealer(io_clone.clone())
            .timed_move_seal(SEALER_MOVE_TIME)
            .await;
        release_bag_from_sealer(io_clone.clone()).await;
    });

    pull_after_flight(io).await;

    state.clear_failures();
    state.set_run_state(RyoRunState::Ready);
    state
}

async fn hmi(io: RyoIo) {
    let shutdown = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");
    info!("HMI Ready");

    hmi::ui_server(
        SocketAddr::from(([0, 0, 0, 0], 3000)),
        io.clone(),
        shutdown.clone(),
    )
    .await
    .unwrap();
    drop(io);
}

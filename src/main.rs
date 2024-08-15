use crate::app_integration::RyoFirebaseClient;
use crate::bag_handler::BagHandler;
use crate::config::*;
use crate::hmi::ui_server_with_fb;
use crate::ryo::RyoRunState::Faulted;
use crate::ryo::{
    drop_bag_sequence, dump_from_hatch, make_bag_handler, make_bag_sensor, make_dispense_tasks,
    make_gantry, pull_before_flight, BagFilledState, BagState, NodeState, RyoFailure, RyoIo,
    RyoRunState, RyoState,
};
use crate::sealer::{sealer, SealerCmd};
use crate::state_server::serve_weights;
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
use control_components::subsystems::dispenser::DispenseParameters;
use tokio::sync::mpsc::Sender;
use tokio::sync::Mutex;
use tokio::task::{spawn_blocking, JoinHandle, JoinSet};
use tokio::time::{interval, sleep};

pub mod config;

pub mod app_integration;
pub mod hmi;
pub mod recipe_handling;

pub mod bag_handler;
pub mod manual_control;
pub mod ryo;
pub mod sealer;
pub mod state_server;

type CCController = clear_core::Controller;
type EtherCATIO = ek1100_io::Controller;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::Builder::from_env(Env::default().default_filter_or("info")).init();
    // console_subscriber::init();
    // let host = env::args()
    //     .nth(1)
    //     .expect("Is this running locally or on Ryo?");

    let task = env::args()
        .nth(1)
        .expect("Do you want to run a cycle or hmi?");

    let run_state = match task.as_str() {
        "cycle" => RyoRunState::Ready,
        "hmi" => RyoRunState::UI,
        _ => RyoRunState::NewJob,
    };

    // TODO: this is a stupid fix to handled single ingredients but will fix next week
    let mut is_single_ingredient = false;
    let recipe = match env::args().nth(2) {
        Some(rec) => match rec.as_str() {
            "potato" => POTATO_HASH_RECIPE,
            "cav" => PESTO_CAVATAPPI_RECIPE,
            "salad" => GARDEN_SALAD_RECIPE,
            "shrimp" => {
                is_single_ingredient = true;
                SHRIMP_RECIPE
            }
            "noods" => {
                is_single_ingredient = true;
                LONG_PASTA_RECIPE
            }
            "tortelloni_all" => {
                is_single_ingredient = true;
                TORTELLONI_ALL_RECIPE
            }
            "tortelloni" => {
                is_single_ingredient = true;
                if let Some(node) = env::args().nth(3) {
                    let mut r: [Option<DispenseParameters>; 4] = array::from_fn(|_| None);
                    match node.as_str() {
                        "0" => {
                            r[0] = Some(TORTELLONI_PARAMETERS);
                        }
                        "1" => {
                            r[1] = Some(TORTELLONI_PARAMETERS);
                        }
                        "2" => {
                            r[2] = Some(TORTELLONI_PARAMETERS);
                        }
                        "3" => {
                            r[3] = Some(TORTELLONI_PARAMETERS);
                        }
                        _ => {
                            error!("Incorrect command line inputs. Try again");
                            return Ok(())
                        }
                    }
                    r
                } else {
                    TORTELLONI_RECIPE
                }

            }
            "usa" => {
                is_single_ingredient = false;
                USA_OMELETTE_RECIPE
            }
            _ => TIMED_RECIPE,
        },
        None => TIMED_RECIPE,
    };
    
    //TODO: Change so that interface can be defined as a compiler flag passed at compile time
    // Figure out a way to detect at launch
    
    let interface = || RYO_INTERFACE;

    let mut io_set = JoinSet::new();

    let scales_handles: [JoinHandle<Scale>; NUMBER_OF_NODES] = array::from_fn(|scale_id| {
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
        io_set.spawn(actor);
        // info!("Spawned {phidget_id} client-actor");
        tx
    });

    let (sealer_tx, sealer_rx) = tokio::sync::mpsc::channel(10);

    //Create IO controllers and their relevant clients
    let (cc1, cl1) = CCController::with_client(CLEAR_CORE_1_ADDR, CC1_MOTORS.as_slice());
    let (cc2, cl2) = CCController::with_client(CLEAR_CORE_2_ADDR, CC2_MOTORS.as_slice());
    let (etc_io, cl3) = EtherCATIO::with_client(interface(), ETHERCAT_NUMBER_OF_SLOTS);

    io_set.spawn(cl3);
    sleep(Duration::from_secs(2)).await;
    io_set.spawn(cl1);
    io_set.spawn(cl2);

    info!("Controller-Client pairs created successfully");

    let ryo_io = RyoIo {
        cc1,
        cc2,
        etc_io,
        scale_txs,
        sealer_tx,
    };

    let sealer_io = ryo_io.clone();
    let sealer_handle = tokio::spawn(async move {
        sealer(sealer_io, sealer_rx).await;
    });

    let gantry = make_gantry(ryo_io.cc1.clone()).await;
    gantry.clear_alerts().await;
    let _ = gantry.enable().await;
    sleep(Duration::from_secs(10)).await;
    info!("Connecting to Firebase");
    let mut firebase = RyoFirebaseClient::new();
    let app_state = Arc::new(Mutex::new(app_integration::Status::default()));
    let mut state;
    let system_mode = Arc::new(Mutex::new(app_integration::SystemMode::UI));
    let shutdown = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
        .expect("Register hook");
    let app_state_for_fb = app_state.clone();
    let system_mode_for_fb = system_mode.clone();
    let shutdown_app = shutdown.clone();
    let app_scales = ryo_io.scale_txs.clone();
    let app_handler = tokio::spawn(async move {
        firebase
            .update(
                app_scales.as_slice(),
                app_state_for_fb,
                system_mode_for_fb,
                shutdown_app,
            )
            .await;
    });

    let weight_server_txs = ryo_io.scale_txs.clone();
    let weight_server_shutdown = shutdown.clone();
    let weight_server = tokio::spawn(async move {
        serve_weights(
            weight_server_txs.as_slice(),
            Arc::clone(&weight_server_shutdown),
        )
        .await
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

    info!("Starting main loop");

    let mut ryo_state = RyoState::new_with_recipe(recipe);
    // TODO: this is a stupid fix 
    if is_single_ingredient {
        ryo_state.set_is_single_ingredient(true);
    }
    ryo_state.set_run_state(run_state);

    let mut loop_interval = interval(Duration::from_millis(100));
    loop {
        if shutdown.load(Ordering::Relaxed) {
            break;
        }
        ryo_state.check_failures();
        match ryo_state.get_run_state() {
            RyoRunState::NewJob => {
                info!("Starting cycle");
                // TODO: don't need this, refactor later
                let _ = pull_before_flight(ryo_io.clone()).await;
                ryo_state.set_run_state(RyoRunState::Running);
            }
            RyoRunState::Running => {
                ryo_state = single_cycle(ryo_state, ryo_io.clone()).await;
            }
            RyoRunState::UI => {
                hmi_with_fb(ryo_io.clone(), ryo_state.clone()).await;
                ryo_state = app_state
                    .lock()
                    .await
                    .update_ryo_state(ryo_state, system_mode.clone())
                    .await;
            }
            RyoRunState::Ready | RyoRunState::Faulted => {
                ryo_state = app_state
                    .lock()
                    .await
                    .update_ryo_state(ryo_state, system_mode.clone())
                    .await;
            }
        }

        loop_interval.tick().await;
    }

    let _ = app_handler.await;
    let _ = weight_server.await;
    let _ = sealer_handle.await;
    while (io_set.join_next().await).is_some() {}
    Ok(())
}

pub enum CycleCmd {
    Cycle(usize),
    Pause,
    Cancel,
}

async fn single_cycle(mut state: RyoState, io: RyoIo) -> RyoState {
    state.update_node_levels(io.clone()).await;
    if let RyoRunState::Faulted = state.get_run_state() {
        error!("All nodes are empty");
        return state;
    }

    info!("Ryo State: {:?}", state);

    let mut dispense_tasks = Vec::new();
    match state.get_bag_state() {
        // TODO: make this fault instead of handling
        BagState::Bagful(BagFilledState::Filled) => {
            info!("Bag already filled")
        }
        BagState::Bagful(BagFilledState::Filling)
        | BagState::Bagful(BagFilledState::Empty)
        | BagState::Bagless => {
            info!("Bag not full, dispensing");
            (state, dispense_tasks) = make_dispense_tasks(state.clone(), io.clone());
        }
    }

    match state.get_bag_state() {
        BagState::Bagless => {
            info!("Getting bag");
            make_bag_handler(io.clone()).dispense_bag().await;
            let gantry = make_gantry(io.cc1.clone()).await;
            let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
            gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
            make_bag_handler(io.clone()).load_bag().await;
            // TODO: maybe have above return results so we know whether to update states?
            state.set_bag_state(BagState::Bagful(BagFilledState::Filling));
        }
        BagState::Bagful(_) => {
            info!("Bag already loaded");
        }
    }
    match state.get_bag_state() {
        BagState::Bagful(BagFilledState::Filled) => {
            info!("Bag already filled, continuing on");
        }
        BagState::Bagful(_) => {
            info!("No bag, getting one");
            let gantry = make_gantry(io.cc1.clone()).await;
            let _ = gantry.absolute_move(GANTRY_NODE_POSITIONS[0]).await;
            let _ = gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
            // TODO: add bag check bypass here ?
            match make_bag_sensor(io.clone()).check().await {
                BagSensorState::Bagless => {
                    warn!("Lost bag");
                    state.set_bag_state(BagState::Bagless);
                    state.log_failure(RyoFailure::BagDispenseFailure);
                    let _ = join_all(dispense_tasks).await;
                    return state;
                }
                BagSensorState::Bagful => (),
            }
        }
        BagState::Bagless => {
            warn!("hmmm this state should not be possible")
        }
    }

    let _ = join_all(dispense_tasks).await;

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
                        if !state.get_is_single_ingredient() {
                            error!("Node {:?} is empty", node);
                            state.set_run_state(Faulted);
                            return state;
                        }
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

    drop_bag_sequence(io.clone()).await;

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

    let _ = make_gantry(io.cc1.clone())
        .await
        .absolute_move(GANTRY_HOME_POSITION)
        .await;
    let _ = io.sealer_tx.send(SealerCmd::Seal).await;
    state.clear_failures();
    // TODO: prob put it back in ready and up a counter of cycles run? will work on with firebase integration
    state.set_run_state(RyoRunState::Running);
    info!("Bag complete!");
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

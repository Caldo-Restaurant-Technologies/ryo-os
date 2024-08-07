use crate::app_integration::RyoFirebaseClient;
use crate::bag_handler::BagHandler;
use crate::config::*;
use crate::hmi::ui_server;
use crate::ryo::RyoRunState::Faulted;
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
use control_components::controllers::ek1100_io::Controller;
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
    let recipe = match env::args().nth(1) {
        Some(rec) => match rec.as_str() {
            "potato" => POTATO_HASH_RECIPE,
            "cav" => PESTO_CAVATAPPI_RECIPE,
            _ => TIMED_RECIPE,
        },
        None => TIMED_RECIPE,
    };

    let mut io_set = JoinSet::new();

    let shutdown = Arc::new(AtomicBool::new(false));
    
    let mut scale  = Scale::new(SINGLE_NODE_PHIDGET_SN);
    scale = Scale::change_coefficients(scale, SINGLE_NODE_COEFFICIENTS.to_vec());
    scale = scale.connect().unwrap();
    let (scale_tx, actor) = scale.actor_tx_pair();
    io_set.spawn(actor);

    //Create IO controllers and their relevant clients
    let (cc, cl) = CCController::with_client(SINGLE_CC_IP_ADDR, &[NODE_MOTOR]);

    io_set.spawn(cl);

    info!("Controller-Client pairs created successfully");

    let weight_server_tx = scale_tx.clone();
    let weight_server_shutdown = shutdown.clone();
    let weight_server = tokio::spawn(async move {
        serve_weights(
            &[weight_server_tx],
            Arc::clone(&weight_server_shutdown),
        )
        .await
    });

    info!("Starting main loop");

    let mut loop_interval = interval(Duration::from_millis(100));
    loop {
        if shutdown.load(Ordering::Relaxed) {
            break;
        }
        ui_server(
            SocketAddr::from(([0, 0, 0, 0], 3000)),
            cc.clone(),
            scale_tx.clone(),
            shutdown.clone(),
            ).await?;
        loop_interval.tick().await;
    }

    let _ = weight_server.await;
    while (io_set.join_next().await).is_some() {}
    Ok(())
}


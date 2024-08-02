use crate::bag_handler::BagHandler;
use crate::config::{GANTRY_HOME_POSITION, GANTRY_SAMPLE_INTERVAL};
use crate::manual_control::{
    disable_all, enable_and_clear_all, handle_dispenser_req, handle_gantry_position_req,
    handle_gantry_req, handle_gripper_req, handle_hatch_position_req, handle_hatch_req,
    handle_hatches_req, handle_sealer_position_req, handle_sealer_req,
};
use crate::ryo::{
    make_bag_handler, make_bag_sensor, make_gantry, pull_before_flight, RyoIo, RyoState,
};
use crate::single_cycle;
use bytes::{Buf, Bytes};
use control_components::components::scale::ScaleCmd;
use control_components::controllers::{clear_core, ek1100_io};
use control_components::subsystems::bag_handling::BagSensorState;
use control_components::subsystems::gantry::GantryCommand;
use control_components::subsystems::node::NodeCommand;
use control_components::util::utils::ascii_to_int;
use http_body_util::{combinators::BoxBody, BodyExt, Empty, Full};
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper::{Method, Request, Response, StatusCode};
use hyper_util::rt::TokioIo;
use log::info;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use tokio::net::{TcpListener, ToSocketAddrs};
use tokio::sync::mpsc::Sender;

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
enum Config {
    Auto,
    Manual,
    Action,
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Step {
    confirm_configuration: Config,
    load_detect: bool,
    position: u8,
    status: bool,
    step_description: &'static str,
    step_id: &'static str,
    step_tutorial: &'static str,
    title: &'static str,
    visible_check: bool,
}

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
pub struct JobSetupStep {
    job_setup_step: HashMap<String, Step>,
}

pub type JobProgress = u32;

pub enum NodeLevel {
    Loaded,
    Medium,
    Low,
    Empty,
}

pub enum TunnelState {
    ConveyorLoaded,
    TunnelLoaded,
    NoTunnel,
}

pub struct NodeWeight {
    pub raw: u32,
    pub scaled: f32,
}

pub struct Node {
    pub tunnel_state: TunnelState,
    pub level: NodeLevel,
    pub weight: NodeWeight, //time_loaded
                            //time_unloaded
}

#[derive(Debug, Clone)]
pub enum HmiState {
    Start,
    Stop,
}

pub enum ManualCmd {
    Gripper(String),
    LoadBag,
}

pub struct OperationSenders {
    node: Sender<NodeCommand>,
    gantry: Sender<GantryCommand>,
}
#[derive(Clone)]
pub struct IOControllers {
    pub cc1: clear_core::Controller,
    pub cc2: clear_core::Controller,
    pub etc: ek1100_io::Controller,
    pub scale_senders: [Sender<ScaleCmd>; 4],
}

type HTTPResult = Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error>;
type HTTPRequest = Request<hyper::body::Incoming>;

pub async fn ui_request_handler(req: HTTPRequest, io: RyoIo, ryo_state: RyoState) -> HTTPResult {
    match (req.method(), req.uri().path()) {
        (&Method::OPTIONS, _) => {
            let response = Response::builder()
                .status(204)
                .header("Access-Control-Allow-Origin", "*")
                .header("Access-Control-Allow-Methods", "POST, OPTIONS, GET, PUT")
                .header("Access-Control-Allow-Headers", "*")
                .body(full("Granting Control"))
                .unwrap();
            Ok(response)
        }
        (&Method::GET, "/") => Ok(Response::new(full("Hola, soy Ryo!"))),
        (&Method::GET, "/job_progress") => Ok(Response::new(full("WIP"))),
        (&Method::GET, "/v1/api/recipe/all") => Ok(Response::new(full("WIP"))),
        (&Method::POST, "/echo") => Ok(Response::new(req.into_body().boxed())),
        (&Method::POST, "/cycle") => {
            let _ = pull_before_flight(io.clone()).await;
            single_cycle(ryo_state, io).await;
            info!("Cycle complete");
            Ok(Response::new(req.into_body().boxed()))
        }
        (&Method::POST, "/gripper") => {
            let body = req.collect().await?.to_bytes();
            let bag_handler = make_bag_handler(io);
            handle_gripper_req(body, bag_handler).await;
            Ok(Response::new(full("Gripper Moved")))
        }
        (&Method::POST, "/load_bag") => {
            let mut bag_handler = BagHandler::new(io.clone());
            bag_handler.dispense_bag().await;
            let gantry = make_gantry(io.cc1).await;
            let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
            let _ = gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
            bag_handler.load_bag().await;
            Ok(Response::new(req.into_body().boxed()))
        }
        (&Method::POST, "/dispense_bag") => {
            BagHandler::new(io).dispense_bag().await;
            Ok(Response::new(req.into_body().boxed()))
        }
        (&Method::POST, "/sealer") => {
            let body = req.collect().await?.to_bytes();
            handle_sealer_req(body, io).await;
            Ok(Response::new(full("Sealer actuated")))
        }
        (&Method::POST, "/sealer_position") => {
            let body = req.collect().await?.to_bytes();
            handle_sealer_position_req(body, io).await;
            Ok(Response::new(full("Sealer to position")))
        }
        (&Method::POST, "/hatch") => {
            let body = req.collect().await?.to_bytes();
            handle_hatch_req(body, io, None).await;
            Ok(Response::new(full("Hatch Moved")))
        }
        (&Method::POST, "/hatches/all") => {
            let body = req.collect().await?.to_bytes();
            handle_hatches_req(body, io).await;
            Ok(Response::new(full("All Hatches Moved")))
        }
        (&Method::POST, "/hatch_position") => {
            let body = req.collect().await?.to_bytes();
            handle_hatch_position_req(body, io).await;
            Ok(Response::new(full("Hatch to position")))
        }
        (&Method::POST, "/gantry") => {
            let body = req.collect().await?.to_bytes();
            let gantry_position = ascii_to_int(body.as_ref()) as usize;
            handle_gantry_req(gantry_position, io).await;
            Ok(Response::new(full("Gantry to position")))
        }
        (&Method::POST, "/gantry_position") => {
            let body = req.collect().await?.to_bytes();
            handle_gantry_position_req(body, io).await;
            Ok(Response::new(full("Gantry to position")))
        }
        (&Method::POST, "/dispense") => {
            let body = req.collect().await?.aggregate();
            // warn!("DEBUG: {:?}", body.chunk());
            let params_json: serde_json::Value = serde_json::from_reader(body.reader()).unwrap();
            handle_dispenser_req(params_json, io).await;
            Ok(Response::new(full("Dispensed")))
        }
        (&Method::POST, "/enable") => {
            enable_and_clear_all(io).await;
            Ok(Response::new(full("Enabled all")))
        }
        (&Method::POST, "/disable") => {
            disable_all(io).await;
            Ok(Response::new(full("Disabled all")))
        }
        (&Method::POST, "/bag_check") => {
            match make_bag_sensor(io).check().await {
                BagSensorState::Bagful => {
                    info!("Bagful!")
                }
                BagSensorState::Bagless => {
                    info!("Bagless!")
                }
            }
            Ok(Response::new(full("Checked Bag State")))
        }
        (_, _) => {
            let mut not_found = Response::new(empty());
            *not_found.status_mut() = StatusCode::NOT_FOUND;
            Ok(not_found)
        }
    }
}

// pub async fn ui_server<T: ToSocketAddrs>(
//     addr: T,
//     controllers: RyoIo,
//     shutdown: Arc<AtomicBool>,
// ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
//     let listener = TcpListener::bind(addr).await?;
//     loop {
//         info!("UI Loop");
//         if shutdown.load(Ordering::Relaxed) {
//             break;
//         }
//         let (stream, _) = listener.accept().await?;
//         let io = TokioIo::new(stream);
//         let controller = controllers.clone();
//         // Spawn a tokio task to serve multiple connections concurrently
//         tokio::task::spawn(async move {
//             // Finally, we bind the incoming connection to our `hello` service
//             if let Err(err) = http1::Builder::new()
//                 // `service_fn` converts our function in a `Service`
//                 .serve_connection(
//                     io,
//                     service_fn(|req| ui_request_handler(req, controller.clone())),
//                 )
//                 .await
//             {
//                 eprintln!("Error serving connection: {:?}", err);
//             }
//         });
//     }
//     Ok(())
// }

pub async fn ui_server_with_fb<T: ToSocketAddrs>(
    addr: T,
    controllers: RyoIo,
    ryo_state: RyoState,
    shutdown: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let listener = TcpListener::bind(addr).await?;
    info!("UI Loop");
    if shutdown.load(Ordering::Relaxed) {
        return Ok(());
    }
    let (stream, _) = listener.accept().await?;
    let io = TokioIo::new(stream);
    let controller = controllers.clone();
    // Spawn a tokio task to serve multiple connections concurrently
    tokio::task::spawn(async move {
        // Finally, we bind the incoming connection to our `hello` service
        if let Err(err) = http1::Builder::new()
            // `service_fn` converts our function in a `Service`
            .serve_connection(
                io,
                service_fn(|req| ui_request_handler(req, controller.clone(), ryo_state.clone())),
            )
            .await
        {
            eprintln!("Error serving connection: {:?}", err);
        }
    });
    Ok(())
}

pub fn empty() -> BoxBody<Bytes, hyper::Error> {
    Empty::<Bytes>::new()
        .map_err(|never| match never {})
        .boxed()
}

pub fn full<T: Into<Bytes>>(chunk: T) -> BoxBody<Bytes, hyper::Error> {
    Full::new(chunk.into())
        .map_err(|never| match never {})
        .boxed()
}

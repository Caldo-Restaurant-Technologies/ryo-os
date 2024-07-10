use crate::bag_handler::{load_bag, BagHandlingCmd, ManualBagHandlingCmd};
use crate::recipe_handling::get_sample_recipe;
use bytes::{Buf, Bytes};
use control_components::controllers::{clear_core, ek1100_io};
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper};
use control_components::subsystems::gantry::GantryCommand;
use control_components::subsystems::linear_actuator::{RelayHBridge, SimpleLinearActuator};
use control_components::subsystems::node::{DispensingParameters, NodeCommand};
use control_components::util::utils::ascii_to_int;
use http_body_util::{combinators::BoxBody, BodyExt, Empty, Full};
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper::{Method, Request, Response, StatusCode};
use hyper_util::rt::TokioIo;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::Duration;
use tokio::net::TcpListener;
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
    pub op_senders: (
        Sender<NodeCommand>,
        Sender<GantryCommand>,
        Sender<ManualBagHandlingCmd>,
    ),
}

type HTTPResult = Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error>;
type HTTPRequest = Request<hyper::body::Incoming>;

pub async fn ui_request_handler(req: HTTPRequest, io: IOControllers) -> HTTPResult {
    match (req.method(), req.uri().path()) {
        (&Method::GET, "/") => Ok(Response::new(full("Hola, soy Ryo!"))),
        (&Method::GET, "/job_progress") => Ok(Response::new(full("WIP"))),
        (&Method::GET, "/v1/api/recipe/all") => Ok(Response::new(full("WIP"))),
        (&Method::POST, "/echo") => Ok(Response::new(req.into_body().boxed())),
        (&Method::POST, "/gripper") => Ok(Response::new(full("WIP"))),
        (&Method::POST, "/load_bag") => Ok(Response::new(req.into_body().boxed())),
        (&Method::POST, "/hatch") => Ok(Response::new(full("WIP"))),
        (&Method::POST, "/hatches/all") => {
            let mut response = full("Ok!");
            let body = req.collect().await?.to_bytes();

            Ok(Response::new(response))
        }
        (&Method::POST, "/gantry") => {
            let body = req.collect().await?.to_bytes();
            let param = ascii_to_int(body.as_ref()) as usize;
            let positions = [-0.25, 24.5, 47.0, 69.5, 92.0];
            let mut response = full("Ok!");
            // if param < positions.len() {
            //     let msg = GantryCommand::GoTo(positions[param]);
            //     if let Err(_) = tx_s.op_senders.1.send(msg).await {
            //         response = full("Sender disconnected");
            //     }
            // } else {
            //     response = full("invalid index");
            // }
            Ok(Response::new(response))
        }
        (&Method::POST, "/dispense") => {
            let _ = req.collect().await?.aggregate();
            //let data: serde_json::Value = serde_json::from_reader(res.reader()).unwrap();
            let sample_recipe = get_sample_recipe();
            let weight = sample_recipe
                .get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .first()
                .unwrap()
                .weight;
            let motor_speed = sample_recipe
                .get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .first()
                .unwrap()
                .motor_speed;
            let sample_rate = sample_recipe
                .get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .first()
                .unwrap()
                .sample_rate;
            let cutoff_freq = sample_recipe
                .get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .first()
                .unwrap()
                .cutoff_frequency;
            let check_offset = sample_recipe
                .get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .first()
                .unwrap()
                .check_offset;
            let stop_offset = sample_recipe
                .get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .first()
                .unwrap()
                .stop_offset;

            let params = DispensingParameters::only_timeout(
                Duration::from_secs_f64(10.0),
                motor_speed,
                sample_rate,
                cutoff_freq,
                check_offset,
                stop_offset,
            );

            let msg = NodeCommand::Dispense(params);
            // tx_s.op_senders.0.send(msg).await.unwrap();
            //println!("{:?}", data);
            Ok(Response::new(full("Dispensing")))
        }
        (_, _) => {
            let mut not_found = Response::new(empty());
            *not_found.status_mut() = StatusCode::NOT_FOUND;
            Ok(not_found)
        }
    }
}

pub async fn ui_server(
    controllers: IOControllers,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let addr = SocketAddr::from(([0, 0, 0, 0], 3000));
    let listener = TcpListener::bind(addr).await?;
    loop {
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
                    service_fn(|req| ui_request_handler(req, controller.clone())),
                )
                .await
            {
                eprintln!("Error serving connection: {:?}", err);
            }
        });
    }
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

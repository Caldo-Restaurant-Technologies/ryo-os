use std::collections::HashMap;
use std::net::SocketAddr;
use bytes::Bytes;
use http_body_util::{Full, combinators::BoxBody, BodyExt, Empty};
use hyper::{Method, Request, Response, StatusCode};
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper_util::rt::TokioIo;
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tokio::sync::mpsc;
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::components::clear_core_io::DigitalInput;
use control_components::controllers::clear_core::Message;

#[derive(Serialize, Deserialize)]
#[serde(rename_all="UPPERCASE")]
enum Config {
    Auto,
    Manual,
    Action
}

#[derive(Serialize, Deserialize)]
#[serde(rename_all="camelCase")]
struct Step {
    confirm_configuration: Config,
    load_detect: bool,
    position: u8,
    status: bool,
    step_description: &'static str,
    step_id: &'static str,
    step_tutorial: &'static str,
    title: &'static str,
    visible_check: bool
}

#[derive(Serialize)]
#[serde(rename_all="camelCase")]
pub struct JobSetupStep {
    job_setup_step: HashMap<String, Step>
}


pub type JobProgress = u32;


pub enum NodeLevel {
    Loaded,
    Medium,
    Low,
    Empty
}

pub enum TunnelState {
    ConveyorLoaded,
    TunnelLoaded,
    NoTunnel
}

pub struct NodeWeight {
    pub raw: u32,
    pub scaled: f32
}


pub struct Node {
    pub tunnel_state: TunnelState,
    pub level: NodeLevel,
    pub weight: NodeWeight
    //time_loaded
    //time_unloaded
}


#[derive(Debug, Clone)]
pub enum HmiState {
    Start,
    Stop,
}

#[derive(Debug, Clone)]
pub struct UISenders{
    pub hmi_state: mpsc::Sender<HmiState>,
    pub drive_senders: Vec<mpsc::Sender<Message>>
}

pub async fn ui_request_handler(
    req: Request<hyper::body::Incoming>,
    tx_s: UISenders
) -> Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error> {
    match (req.method(), req.uri().path()) {
        (&Method::GET, "/") => Ok(Response::new(full("Hola, soy Ryo!"))),
        (&Method::GET, "/job_progress") => {
            Ok(Response::new(full("WIP")))
        }
        (&Method::GET, "/motor_state") => {
            let mut states = Vec::new();
            for drive in tx_s.drive_senders{
                for i in 0..4 {
                    let state = ClearCoreMotor::new(i, 800, drive.clone())
                        .get_status()
                        .await
                        .unwrap();
                    states.push(state);
                }
            }
            let serialized_statuses = serde_json::to_string(&states).unwrap();
            Ok(Response::new(full(serialized_statuses)))
        }
        (&Method::GET, "/input_state") => {
            let mut states = Vec::new();
            for drive in tx_s.drive_senders{
                for i in 0..6 {
                    let state = DigitalInput::new(i, drive.clone())
                        .get_state()
                        .await
                        .unwrap();
                    states.push(state);
                }
            }
            let serialized_statuses = serde_json::to_string(&states).unwrap();
            Ok(Response::new(full(serialized_statuses)))
        }
        (&Method::GET, "/v1/api/recipe/all") => {Ok(Response::new(full("WIP")))},
        (&Method::POST, "/echo") => { Ok(Response::new(req.into_body().boxed()))},
        (&Method::POST, "/drive_command") => { Ok(Response::new(req.into_body().boxed()))},
        (&Method::POST, "/dispense") => { Ok(Response::new(req.into_body().boxed()))},
        (_, _) => {
            let mut not_found = Response::new(empty());
            *not_found.status_mut() = StatusCode::NOT_FOUND;
            Ok(not_found)
        }
    }
}

pub async fn ui_server(
    txs: UISenders,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let addr = SocketAddr::from(([0, 0, 0, 0], 3000));
    let listener = TcpListener::bind(addr).await?;
    loop {
        let (stream, _) = listener.accept().await?;
        let io = TokioIo::new(stream);
        let txs = txs.clone();
        // Spawn a tokio task to serve multiple connections concurrently
        tokio::task::spawn(async move {
            // Finally, we bind the incoming connection to our `hello` service
            if let Err(err) = http1::Builder::new()
                // `service_fn` converts our function in a `Service`
                .serve_connection(io, service_fn(|req| ui_request_handler(req, txs.clone())))
                .await
            {
                eprintln!("Error serving connection: {:?}", err);
            }
        });
    }
}

fn empty() -> BoxBody<Bytes, hyper::Error> {
    Empty::<Bytes>::new().map_err(|never|{match never{}}).boxed()
}

fn full<T: Into<Bytes>>(chunk: T) -> BoxBody<Bytes, hyper::Error> {
    Full::new(chunk.into())
        .map_err(|never| match never {})
        .boxed()
}
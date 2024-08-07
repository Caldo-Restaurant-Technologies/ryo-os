use crate::manual_control::{
    disable_all, enable_and_clear_all, handle_dispenser_req,
};
use crate::CCController;
use bytes::{Buf, Bytes};
use control_components::components::scale::ScaleCmd;
use control_components::controllers::clear_core;
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
use control_components::controllers::clear_core::Controller;
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

type HTTPResult = Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error>;
type HTTPRequest = Request<hyper::body::Incoming>;

pub async fn ui_request_handler(req: HTTPRequest, cc: CCController, scale_tx: Sender<ScaleCmd>) -> HTTPResult {
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
        (&Method::POST, "/echo") => Ok(Response::new(req.into_body().boxed())),
        (&Method::POST, "/dispense") => {
            let body = req.collect().await?.aggregate();
            // warn!("DEBUG: {:?}", body.chunk());
            let params_json: serde_json::Value = serde_json::from_reader(body.reader()).unwrap();
            handle_dispenser_req(params_json, cc, scale_tx).await;
            Ok(Response::new(full("Dispensed")))
        }
        (&Method::POST, "/enable") => {
            enable_and_clear_all(cc).await;
            Ok(Response::new(full("Enabled all")))
        }
        (&Method::POST, "/disable") => {
            disable_all(cc).await;
            Ok(Response::new(full("Disabled all")))
        }
        (_, _) => {
            let mut not_found = Response::new(empty());
            *not_found.status_mut() = StatusCode::NOT_FOUND;
            Ok(not_found)
        }
    }
}

pub async fn ui_server<T: ToSocketAddrs>(
    addr: T,
    cc: CCController,
    scale_tx: Sender<ScaleCmd>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let listener = TcpListener::bind(addr).await?;
    loop {
        info!("UI Loop");
        if shutdown.load(Ordering::Relaxed) {
            break;
        }
        let (stream, _) = listener.accept().await?;
        let io = TokioIo::new(stream);
        let controller = cc.clone();
        let tx = scale_tx.clone();
        // Spawn a tokio task to serve multiple connections concurrently
        tokio::task::spawn(async move {
            // Finally, we bind the incoming connection to our `hello` service
            if let Err(err) = http1::Builder::new()
                // `service_fn` converts our function in a `Service`
                .serve_connection(
                    io,
                    service_fn(|req| ui_request_handler(req, controller.clone(), tx.clone())),
                )
                .await
            {
                eprintln!("Error serving connection: {:?}", err);
            }
        });
    }
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

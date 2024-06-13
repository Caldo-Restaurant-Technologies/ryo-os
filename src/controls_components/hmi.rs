use std::collections::HashMap;
use std::convert::Infallible;
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
struct JobSetupStep {
    job_setup_step: HashMap<String, Step>
}


type JobProgress = u32;

enum NodeLevel {
    Loaded,
    Medium,
    Low,
    Empty
}

enum TunnelState {
    ConveyorLoaded,
    TunnelLoaded,
    NoTunnel
}

struct NodeWeight {
    raw: u32,
    scaled: f32
}


struct Node {
    tunnel_state: TunnelState,
    level: NodeLevel,
    weight: NodeWeight
    //time_loaded
    //time_unloaded
}

type NodeArray<Nodes, const N: usize> = [Nodes;N];

#[derive(Debug, Clone)]
pub enum HmiState {
    Start,
    Stop,
}

async fn hello(
    _: Request<hyper::body::Incoming>,
    tx: mpsc::Sender<HmiState>,
) -> Result<Response<Full<Bytes>>, Infallible> {
    tx.send(HmiState::Start).await.unwrap();
    Ok(Response::new(Full::new(Bytes::from("Hello, World!"))))
}

pub async fn ui_request_handler(
    req: Request<hyper::body::Incoming>
) -> Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error> {
    match (req.method(), req.uri().path()) {
        (&Method::GET, "/") => Ok(Response::new(full("Hola, soy Ryo!"))),
        (&Method::POST, "/echo") => { Ok(Response::new(req.into_body().boxed()))}
        (&Method::GET, "/v1/api/recipe/all") => {Ok(Response::new(full("WIP")))},
        (_, _) => {
            let mut not_found = Response::new(empty());
            *not_found.status_mut() = StatusCode::NOT_FOUND;
            Ok(not_found)
        }
    }
}

pub async fn ui_request_handler_(mut ui_state: mpsc::Receiver<HmiState>) {
    while let Some(state) = ui_state.recv().await {
        match state {
            HmiState::Start => {
                println!("Start Command Sent")
            }
            HmiState::Stop => {
                println!("Start Command Sent")
            }
        }
    }
}

pub async fn ui_server(
    tx: mpsc::Sender<HmiState>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let addr = SocketAddr::from(([0, 0, 0, 0], 3000));
    // We create a TcpListener and bind it to 127.0.0.1:3000
    let listener = TcpListener::bind(addr).await?;
    loop {
        let (stream, _) = listener.accept().await?;
        let io = TokioIo::new(stream);
        // Spawn a tokio task to serve multiple connections concurrently
        let tx = tx.clone();
        tokio::task::spawn(async move {
            // Finally, we bind the incoming connection to our `hello` service
            if let Err(err) = http1::Builder::new()
                // `service_fn` converts our function in a `Service`
                .serve_connection(io, service_fn(|req| hello(req, tx.clone())))
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
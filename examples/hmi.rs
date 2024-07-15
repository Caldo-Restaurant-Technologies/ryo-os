use bytes::{Buf, Bytes};
use control_components::components::scale::{Scale, ScaleCmd};
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
use std::{array, env};
use std::error::Error;
use std::net::SocketAddr;
use std::time::Duration;
use control_components::components::clear_core_motor::ClearCoreMotor;
use env_logger::Env;
use futures::future::{err, join_all};
use log::{error, info};
use tokio::net::{TcpListener, ToSocketAddrs};
use tokio::sync::mpsc::{channel, Sender};
use tokio::task::{JoinHandle, JoinSet, spawn_blocking};
use tokio::time::sleep;
use ryo_os::config::{CC1_MOTORS, CC2_MOTORS, CLEAR_CORE_1_ADDR, CLEAR_CORE_2_ADDR};


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

#[derive(Debug, Clone)]
pub enum HmiState {
    Start,
    Stop,
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
type CCController = clear_core::Controller;

#[derive(Clone)]
pub struct ClearCoreControllers {
    cc1: CCController,
    cc2: CCController,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut client_set = JoinSet::new();
    
    //Create IO controllers and their relevant clients
    let (cc1, cl1) = CCController::with_client(CLEAR_CORE_1_ADDR, CC1_MOTORS.as_slice());
    let (cc2, cl2) = CCController::with_client(CLEAR_CORE_2_ADDR, CC2_MOTORS.as_slice());
    let io = ClearCoreControllers {cc1, cc2};
    let addr = SocketAddr::from(([0, 0, 0, 0], 3000));
    
    client_set.spawn(cl1);
    client_set.spawn(cl2);

    info!("Controller-Client pairs created successfully");

    ui_server(addr, io).await.unwrap();

    while let Some(_) = client_set.join_next().await {}
    Ok(())
}

pub async fn ui_request_handler(req: HTTPRequest, io: ClearCoreControllers) -> HTTPResult {
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
            error!("Cycle not yet functional :(");
            Ok(Response::new(req.into_body().boxed()))
        },
        (&Method::POST, "/enable") => {
            let body = req.collect().await?.to_bytes();
            get_motor(body, io).unwrap().enable().await.unwrap();
            Ok(Response::new(full("Motor Enabled")))
        }
        (&Method::POST, "/disable") => {
            let body = req.collect().await?.to_bytes();
            get_motor(body, io).unwrap().disable().await;
            Ok(Response::new(full("Motor Disabled")))
        }
        (&Method::POST, "/clear") => {
            let body = req.collect().await?.to_bytes();
            get_motor(body, io).unwrap().clear_alerts().await;
            Ok(Response::new(full("Motor Alerts Cleared")))
        }
        (&Method::POST, "/jog_forward") => {
            let body = req.collect().await?.to_bytes();
            get_motor(body, io).unwrap().jog(10.).await.unwrap();
            Ok(Response::new(full("Motor Jogging")))
        }
        (&Method::POST, "/jog_backward") => {
            let body = req.collect().await?.to_bytes();
            get_motor(body, io).unwrap().jog(-10.).await.unwrap();
            Ok(Response::new(full("Motor Jogging")))
        }
        (&Method::POST, "/relative_move") => {
            let body = req.collect().await?.to_bytes();
            get_motor(body, io).unwrap().relative_move(100.).await.unwrap();
            Ok(Response::new(full("Motor Moving")))
        }
        (&Method::POST, "/absolute_move") => {
            let body = req.collect().await?.to_bytes();
            get_motor(body, io).unwrap().absolute_move(0.).await.unwrap();
            Ok(Response::new(full("Motor Moving")))
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
    controllers: ClearCoreControllers,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
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

pub fn get_motor(body: Bytes, io: ClearCoreControllers) -> Result<ClearCoreMotor, HMIError> {
    let cc = match body[0] {
        b'1' => io.cc1,
        b'2' => io.cc2,
        _ => {
            error!("Invalid ClearCore ID");
            return Err(HMIError::InvalidCCID)
        }
    };
    let motor_id = match body[1] {
        b'0' => 0,
        b'1' => 1,
        b'2' => 2,
        b'3' => 3,
        _ => {
            error!("Invalid Motor ID");
            return Err(HMIError::InvalidMotorID)
        }
    };
    Ok(cc.get_motor(motor_id))
}

#[derive(Debug)]
pub enum HMIError {
    InvalidCCID,
    InvalidMotorID,
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

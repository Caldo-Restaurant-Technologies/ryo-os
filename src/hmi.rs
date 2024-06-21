use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::Duration;
use bytes::{Buf,Bytes};
use http_body_util::{Full, combinators::BoxBody, BodyExt, Empty};
use hyper::{Method, Request, Response, StatusCode};
use hyper::server::conn::http1;
use hyper::service::service_fn;
use hyper_util::rt::TokioIo;
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tokio::sync::mpsc::Sender;
use control_components::components::clear_core_motor::{ClearCoreMotor, Status};
use control_components::components::clear_core_io::{DigitalInput, Output};
use control_components::controllers::clear_core::Message;
use control_components::subsystems::gantry::GantryCommand;
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{RelayHBridge, SimpleLinearActuator};
use control_components::subsystems::node::{DispensingParameters, NodeCommand};
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper, load_bag};
use control_components::util::utils::ascii_to_int;
use crate::config::{get_inputs, get_motors, RYO_INPUT_COUNT, RYO_MOTOR_COUNT, BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, STEPPER_MOTOR_COUNTS, GRIPPER_MOTOR_ID, GRIPPER_ACTUATOR, CC_STEP_COUNTS, BAG_BLOWER, GANTRY_MOTOR_ID}
;
use crate::recipe_handling::get_sample_recipe;

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
    pub hmi_state: Sender<HmiState>,
    pub drive_senders: (Sender<Message>, Sender<Message>),
    pub op_senders: (Sender<NodeCommand>, Sender<GantryCommand>)
}

async fn get_motor_states(drive_1: Sender<Message>, drive_2: Sender<Message>) -> Vec<Status> {
    let mut states = Vec::with_capacity(RYO_MOTOR_COUNT);
    for motor in get_motors(drive_1, drive_2).as_slice() {
        let status = motor.get_status().await.unwrap();
        states.push(status);
    }
    states
}

async fn get_input_state(drive_1: Sender<Message>) -> Vec<bool> {
    let mut states = Vec::with_capacity(RYO_INPUT_COUNT);
    for input in get_inputs(drive_1).as_slice() {
        let status = input.get_state().await.unwrap();
        states.push(status);
    }
    states
}

pub async fn ui_request_handler(
    req: Request<hyper::body::Incoming>,
    tx_s: UISenders
) -> Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error> {



    let hatch_a_actuator = RelayHBridge::new(
        tx_s.drive_senders.0.clone(), (2,3),3
    );
    let hatch_b_actuator = RelayHBridge::new(
        tx_s.drive_senders.0.clone(), (4,5), 4
    );
    let hatch_c_actuator = RelayHBridge::new(
        tx_s.drive_senders.1.clone(), (0,1),3
    );
    let hatch_d_actuator = RelayHBridge::new(
        tx_s.drive_senders.1.clone(), (2,3), 4
    );




    match (req.method(), req.uri().path()) {
        (&Method::GET, "/") => Ok(Response::new(full("Hola, soy Ryo!"))),
        (&Method::GET, "/job_progress") => {
            Ok(Response::new(full("WIP")))
        }
        (&Method::GET, "/motor_state") => {
            let states = get_motor_states(tx_s.drive_senders.0, tx_s.drive_senders.1).await;
            Ok(Response::new(full(serde_json::to_string(&states).unwrap())))
        }
        (&Method::GET, "/input_state") => {
            let states = get_input_state(tx_s.drive_senders.0).await;
            let serialized_statuses = serde_json::to_string(&states).unwrap();
            Ok(Response::new(full(serialized_statuses)))
        }
        (&Method::GET, "/v1/api/recipe/all") => {Ok(Response::new(full("WIP")))},
        (&Method::POST, "/echo") => { Ok(Response::new(req.into_body().boxed()))},
        (&Method::POST, "/gripper") => {
            let bag_gripper = BagGripper::new(
                ClearCoreMotor::new(GRIPPER_MOTOR_ID, CC_STEP_COUNTS, tx_s.drive_senders.1.clone()),
                SimpleLinearActuator::new(
                    tx_s.drive_senders.1.clone(),
                    GRIPPER_ACTUATOR,
                    0
                ),
                [-0.4, 0.8, -0.4].to_vec()
            );
            let body = req.collect().await?.to_bytes();
            println!("{:?}", body);
            let task = tokio::spawn(async move {
                if body.len() > 0 {
                    if body[0] == b'o'{
                        bag_gripper.open().await.unwrap();
                    } else if body[0] == b'c' {
                        bag_gripper.close().await.unwrap();
                    }
                }
            });
            task.await.unwrap();
            Ok(Response::new(full("WIP")))
        }
        (&Method::POST, "/load_bag") => {
            println!("load bag called!");
            let bag_dispenser =  BagDispenser::new(
                ClearCoreMotor::new(BAG_ROLLER_MOTOR_ID, STEPPER_MOTOR_COUNTS, tx_s.drive_senders.0.clone()),
                DigitalInput::new(BAG_ROLLER_PE, tx_s.drive_senders.0.clone())
            );
            let bag_gripper = BagGripper::new(
                ClearCoreMotor::new(GRIPPER_MOTOR_ID, CC_STEP_COUNTS, tx_s.drive_senders.1.clone()),
                SimpleLinearActuator::new(
                    tx_s.drive_senders.1.clone(),
                    GRIPPER_ACTUATOR,
                    0
                ),
                [-0.4, 0.8, -0.4].to_vec()
            );

            let blower = Output::new(BAG_BLOWER, tx_s.drive_senders.1.clone());
            let task = tokio::spawn(async move {
                let gantry = ClearCoreMotor::new(
                    GANTRY_MOTOR_ID, CC_STEP_COUNTS, tx_s.drive_senders.0.clone()
                );
                gantry.enable().await.unwrap();
                gantry.absolute_move(-0.25).await.unwrap();
                gantry.wait_for_move(Duration::from_secs_f64(0.5)).await.unwrap();
                load_bag(bag_dispenser, bag_gripper, blower).await;
            });
            task.await.unwrap();
            Ok(Response::new(req.into_body().boxed()))
        }
        (&Method::POST, "/hatch") => {
            let mut response = full("Ok!");
            let body = req.collect().await?.to_bytes();

            if body.len() >= 2 {
                let hatch_id = body[0] - 48;
                let operation = body[1];

                let hatch = match hatch_id {
                    0 => {
                        Some(Hatch::new(hatch_a_actuator, Duration::from_secs_f64(3.)))
                    },
                    1 => {
                        Some(Hatch::new(hatch_b_actuator, Duration::from_secs_f64(3.)))
                    },
                    2 => {
                        Some(Hatch::new(hatch_c_actuator, Duration::from_secs_f64(1.7)))
                    },
                    3 => {
                        Some(Hatch::new(hatch_d_actuator, Duration::from_secs_f64(3.)))
                    }
                    _ => {None}
                };
                if hatch.is_some() {
                    if operation == b'o' {
                        hatch.unwrap().timed_open(Duration::from_secs_f64(2.1)).await.unwrap();
                    } else if operation == b'c' {
                        hatch.unwrap().timed_close(Duration::from_secs_f64(2.1)).await.unwrap();
                    }
                } else {
                    response = full("Invalid Request");
                }
            } else {
                response = full("Invalid Request");
            }
            Ok(Response::new(response))
        }
        (&Method::POST, "/hatches/all") => {

            let mut response = full("Ok!");
            let body = req.collect().await?.to_bytes();
            if body.len() > 1{
                if body[0] == b'o' {
                    let _ = tokio::spawn(async move {
                        Hatch::new(hatch_a_actuator, Duration::from_secs_f64(3.))
                            .timed_open(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                        Hatch::new(hatch_b_actuator, Duration::from_secs_f64(3.))
                            .timed_open(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                        Hatch::new(hatch_c_actuator, Duration::from_secs_f64(3.))
                            .timed_open(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                        Hatch::new(hatch_d_actuator, Duration::from_secs_f64(3.))
                            .timed_open(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                    });
                } else if body[0] == b'c' {
                    let _ = tokio::spawn(async move {
                        Hatch::new(hatch_a_actuator, Duration::from_secs_f64(3.))
                            .timed_close(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                        Hatch::new(hatch_b_actuator, Duration::from_secs_f64(3.))
                            .timed_close(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                        Hatch::new(hatch_c_actuator, Duration::from_secs_f64(3.))
                            .timed_close(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                        Hatch::new(hatch_d_actuator, Duration::from_secs_f64(3.))
                            .timed_close(Duration::from_secs_f64(2.1))
                            .await
                            .unwrap();
                    });
                } else {
                    response = full("Wrong operation homie");
                }
            }
            Ok(Response::new(response))
        }
        (&Method::POST, "/gantry") => {
            let body = req.collect().await?.to_bytes();
            let param = ascii_to_int(body.as_ref()) as usize;
            let positions = [-0.25, 24.5, 47.0, 69.5, 92.0];
            let mut response = full("Ok!");
            if param < positions.len() {
                let msg = GantryCommand::GoTo(positions[param]);
                if let Err(_) = tx_s.op_senders.1.send(msg).await {
                    response = full("Sender disconnected");
                }
            } else {
                response = full("invalid index");
            }
            Ok(Response::new(response))
        },
        (&Method::POST, "/dispense") =>{
            let _ = req.collect().await?.aggregate();
            //let data: serde_json::Value = serde_json::from_reader(res.reader()).unwrap();
            let sample_recipe = get_sample_recipe();
            let weight = sample_recipe.get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .get(0)
                .unwrap()
                .weight;
           let motor_speed = sample_recipe.get("0")
               .unwrap()
               .ingredients
               .as_slice()
               .get(0)
               .unwrap()
               .motor_speed;
            let sample_rate = sample_recipe.get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .get(0)
                .unwrap()
                .sample_rate;
            let cutoff_freq = sample_recipe.get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .get(0)
                .unwrap()
                .cutoff_frequency;
            let check_offset = sample_recipe.get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .get(0)
                .unwrap()
                .check_offset;
            let stop_offset = sample_recipe.get("0")
                .unwrap()
                .ingredients
                .as_slice()
                .get(0)
                .unwrap()
                .stop_offset;

            let params = DispensingParameters::only_timeout(
                Duration::from_secs_f64(10.0),
                motor_speed,
                sample_rate,
                cutoff_freq,
                check_offset,
                stop_offset
            );
            
            let msg = NodeCommand::Dispense(params);
            tx_s.op_senders.0.send(msg).await.unwrap();
            //println!("{:?}", data);
            Ok(Response::new(full("Dispensing")))
        },
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
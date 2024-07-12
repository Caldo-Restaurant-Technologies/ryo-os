use crate::bag_handler::{load_bag, BagHandlingCmd, ManualBagHandlingCmd};
use crate::config::{
    DISPENSER_TIMEOUT, GANTRY_ALL_POSITIONS, GANTRY_MOTOR_ID, GANTRY_SAMPLE_INTERVAL,
    HATCHES_OPEN_TIME, HATCH_CLOSE_TIMES,
};
use crate::hmi::{empty, full};
use crate::ryo::{make_dispenser, make_dispensers, make_gripper, make_hatch, make_hatches, RyoIo};
use bytes::{Buf, Bytes};
use control_components::components::clear_core_motor::{ClearCoreMotor, Status};
use control_components::controllers::{clear_core, ek1100_io};
use control_components::subsystems::bag_handling::BagGripper;
use control_components::subsystems::dispenser::{
    DispenseParameters, Dispenser, Parameters, Setpoint, WeightedDispense,
};
use futures::future::join_all;
use http_body_util::{combinators::BoxBody, BodyExt, Empty, Full};
use hyper::{Method, Request, Response, StatusCode};
use hyper_util::rt::TokioIo;
use log::{error, info, warn};
use serde::{Deserialize, Serialize};
use std::array;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::Duration;
use tokio::net::TcpListener;
use tokio::sync::mpsc::Sender;
use tokio::task::JoinHandle;

type HTTPResult = Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error>;
type HTTPRequest = Request<hyper::body::Incoming>;

pub enum ActuatorCmd {
    Open,
    Close,
}
pub enum ManualCmd {
    GetMotorStates,
    GetInputStates,
    Gripper(ActuatorCmd),
    LoadBag,
    DispenseBag,
    HatchCmd(ActuatorCmd),
    GantryCmd(usize),
    Dispense(usize),
    CancelDispense(usize),
}

pub async fn handle_gripper_req(body: Bytes, mut gripper: BagGripper) {
    info!("{:?}", body);
    if body.len() > 0 {
        if body[0] == b'o' {
            info!("Opening Gripper");
            gripper.open().await;
        } else if body[0] == b'c' {
            info!("Closing Gripper");
            gripper.close().await;
        }
    }
}

pub async fn handle_hatch_req(body: Bytes, io: RyoIo, hatch_id: Option<usize>) {
    if body.len() >= 2 {
        let (hatch_id, operation) = match hatch_id {
            Some(hatch_id) => (hatch_id, body[0]),
            None => ((body[0] - 48) as usize, body[1]),
        };
        info!("Hatch {:}", hatch_id);
        let mut hatch = make_hatch(hatch_id, io.cc1, io.cc2);
        if operation == b'o' {
            info!("Opening Hatch {:}", hatch_id);
            hatch.timed_open(HATCHES_OPEN_TIME).await;
        } else if operation == b'c' {
            info!("Closing Hatch {:}", hatch_id);
            hatch.timed_close(HATCH_CLOSE_TIMES[hatch_id]).await;
        }
    }
}

pub async fn handle_hatches_req(body: Bytes, io: RyoIo) {
    let hatch_handles: [JoinHandle<()>; 4] = array::from_fn(|hatch_id| {
        let hatch_io = io.clone();
        let hatch_body = body.clone();
        tokio::spawn(async move { handle_hatch_req(hatch_body, hatch_io, Some(hatch_id)).await })
    });
    join_all(hatch_handles).await;
}

pub async fn handle_gantry_req(gantry_position: usize, io: RyoIo) {
    let gantry_motor = io.cc1.get_motor(GANTRY_MOTOR_ID);
    match gantry_motor
        .absolute_move(GANTRY_ALL_POSITIONS[gantry_position])
        .await
    {
        Ok(_) => (),
        Err(status) => {
            warn!("Gantry Motor Status: {:?}", status);
            match status {
                Status::Disabled => {
                    gantry_motor.enable().await.unwrap();
                }
                Status::Faulted => gantry_motor.clear_alerts().await,
                _ => {
                    error!("Could not handle gantry motor state");
                    return;
                }
            }
        }
    }
    io.cc1
        .get_motor(GANTRY_MOTOR_ID)
        .wait_for_move(GANTRY_SAMPLE_INTERVAL)
        .await;
}

pub async fn handle_dispenser_req(json: serde_json::Value, io: RyoIo) {
    let node_id = json["node_id"]
        .as_str()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap();
    let dispense_type = json["dispense_type"].as_str().unwrap();
    // placeholder
    let timeout = Duration::from_secs(120);
    // let timeout = json["timeout"].as_str().unwrap();
    let serving_weight = json["serving_weight"]
        .as_str()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap();
    let motor_speed = json["motor_speed"]
        .as_str()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap();
    let sample_rate = json["sample_rate"]
        .as_str()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap();
    let cutoff_frequency = json["cutoff_frequency"]
        .as_str()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap();
    let check_offset = json["check_offset"]
        .as_str()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap();
    let stop_offset = json["stop_offset"]
        .as_str()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap();
    let parameters = Parameters {
        motor_speed,
        sample_rate,
        cutoff_frequency,
        check_offset,
        stop_offset,
    };
    make_dispenser(
        node_id,
        io.cc2,
        match dispense_type {
            "timed" => Setpoint::Timed(timeout),
            "weight" => Setpoint::Weight(WeightedDispense {
                setpoint: serving_weight,
                timeout,
            }),
            _ => {
                error!("Invalid Dispense Type");
                return;
            }
        },
        parameters,
        io.scale_txs[node_id].clone(),
    )
    .dispense(DISPENSER_TIMEOUT)
    .await;
    info!("Dispensed from Node {:}", node_id);
}

pub async fn enable_and_clear_all(io: RyoIo) {
    let cc1_motors: [ClearCoreMotor; 3] = array::from_fn(|motor_id| io.cc1.get_motor(motor_id));
    let cc2_motors: [ClearCoreMotor; 4] = array::from_fn(|motor_id| io.cc2.get_motor(motor_id));

    let enable_clear_cc1_handles = cc1_motors.iter().map(|motor| async move {
        motor.clear_alerts().await;
        let _ = motor.enable().await;
    });
    let enable_clear_cc2_handles = cc2_motors.iter().map(|motor| async move {
        motor.clear_alerts().await;
        let _ = motor.enable().await;
    });
    join_all(enable_clear_cc1_handles).await;
    join_all(enable_clear_cc2_handles).await;
}

pub async fn disable_all(io: RyoIo) {
    let cc1_motors: [ClearCoreMotor; 3] = array::from_fn(|motor_id| io.cc1.get_motor(motor_id));
    let cc2_motors: [ClearCoreMotor; 4] = array::from_fn(|motor_id| io.cc2.get_motor(motor_id));

    let disable_cc1_handles = cc1_motors.iter().map(|motor| async move {
        motor.disable().await
    });
    let disable_cc2_handles = cc2_motors.iter().map(|motor| async move {
        motor.disable().await
    });
    join_all(disable_cc1_handles).await;
    join_all(disable_cc2_handles).await;
}

// pub async response_build()

// pub async fn manual_request_handler(req: HTTPRequest, io: RyoIo) -> HTTPResult {
//     match (req.method(), req.uri().path()) {
//         (&Method::OPTIONS, _) => {
//             let response = Response::builder()
//                 .status(204)
//                 .header("Access-Control-Allow-Origin", "*")
//                 .header("Access-Control-Allow-Headers", "*")
//                 .header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
//                 .body(full("test"))
//                 .unwrap();
//             Ok(response)
//         }
//         (&Method::POST, "/gripper") => {
//             let body = req.collect().await?.to_bytes();
//             let gripper = make_gripper(io.cc1, io.cc2);
//             handle_gripper_req(body, gripper).await;
//             Ok(Response::new(full("Gripper request complete")))
//         }
//         (&Method::POST, "/dispense_bag") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/load_bag") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/hatch") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/gantry") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/dispense") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/cancel") => Ok(Response::new(full("WIP"))),
//
//         (_, _) => {
//             let mut not_found = Response::new(empty());
//             *not_found.status_mut() = StatusCode::NOT_FOUND;
//             Ok(not_found)
//         }
//     }
// }

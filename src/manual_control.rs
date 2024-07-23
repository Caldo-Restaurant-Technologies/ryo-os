use crate::bag_handler::{BagHandler, ManualBagHandlingCmd};
use crate::config::{
    GANTRY_ALL_POSITIONS, GANTRY_MOTOR_ID, GANTRY_SAMPLE_INTERVAL,
    SEALER_MOVE_DOOR_TIME,
};
use crate::ryo::{
    make_and_close_hatch, make_and_move_hatch, make_and_open_hatch, make_dispenser,
    make_gantry, make_hatch, make_sealer, make_trap_door, RyoIo,
};
use bytes::{Bytes};
use control_components::components::clear_core_io::HBridgeState;
use control_components::components::clear_core_motor::{ClearCoreMotor, Status};
use control_components::subsystems::dispenser::{
    Parameters, Setpoint, WeightedDispense,
};
use control_components::util::utils::ascii_to_int;
use futures::future::join_all;
use http_body_util::{combinators::BoxBody};
use hyper::{Request, Response};
use log::{error, info, warn};
use std::array;
use std::time::Duration;
use control_components::components::scale::ScaleCmd;
use tokio::task::JoinHandle;
use tokio::time::sleep;

// type HTTPResult = Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error>;
// type HTTPRequest = Request<hyper::body::Incoming>;

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

pub async fn handle_gripper_req(body: Bytes, mut bag_handler: BagHandler) {
    if !body.is_empty() {
        if body[0] == b'o' {
            info!("Opening Gripper");
            bag_handler.open_gripper().await;
        } else if body[0] == b'c' {
            info!("Closing Gripper");
            bag_handler.close_gripper().await;
        }
    }
}

pub async fn handle_hatch_req(body: Bytes, io: RyoIo, hatch_id: Option<usize>) {
    if body.len() >= 2 {
        let (hatch_id, operation) = match hatch_id {
            Some(hatch_id) => (hatch_id, body[0]),
            None => ((body[0] - 48) as usize, body[1]),
        };
        let names = ["A", "B", "C", "D"];
        if operation == b'o' {
            info!("Opening Hatch {:}", names[hatch_id]);
            make_and_open_hatch(hatch_id, io).await;
        } else if operation == b'c' {
            info!("Closing Hatch {:}", names[hatch_id]);
            make_and_close_hatch(hatch_id, io).await;
        }
    }
}

pub async fn handle_hatch_position_req(body: Bytes, io: RyoIo) {
    let hatch_id = match body[0] {
        b'A' => 0,
        b'B' => 1,
        b'C' => 2,
        b'D' => 3,
        _ => {
            error!("Invalid Hatch ID");
            return;
        }
    };
    let names = ["A", "B", "C", "D"];
    match body.len() {
        1 => {
            let pos = make_hatch(hatch_id, io).get_position().await;
            info!("Hatch {:?} at Position {:?}", names[hatch_id], pos);
        }
        _ => {
            let position = ascii_to_int(body[1..].as_ref());

            info!("Hatch {:?} to position {:?}", names[hatch_id], position);
            make_and_move_hatch(hatch_id, position, io.clone()).await;
            let pos = make_hatch(hatch_id, io).get_position().await;
            info!("Hatch {:?} at position {:?}", names[hatch_id], pos);
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
    gantry_motor.set_acceleration(50.).await;
    gantry_motor.set_velocity(150.).await;
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
        .await.unwrap();
    let positions = ["Home", "Node A", "Node B", "Node C", "Node D", "Bag Drop"];
    info!("Gantry to {:}", positions[gantry_position].to_string());
}

pub async fn handle_gantry_position_req(body: Bytes, io: RyoIo) {
    match body.len() {
        0 => {
            let pos = make_gantry(io.cc1).get_position().await;
            info!("Gantry at Position {:?}", pos);
        }
        _ => {
            let position = ascii_to_int(body.as_ref());
            info!("Gantry to position {:?}", position);
            let gantry = make_gantry(io.cc1);
            let _ = gantry.absolute_move(position as f64).await;
            let pos = gantry.get_position().await;
            info!("Gantry at position {:?}", pos);
        }
    }
}

pub async fn handle_dispenser_req(json: serde_json::Value, io: RyoIo) {
    // let node_id = json["node_id"]
    //     .as_str()
    //     .and_then(|s| s.parse::<usize>().ok())
    //     .unwrap();
    let node_id = match json["node_id"].as_str() {
        Some("0") => 0,
        Some("1") => 1,
        Some("2") => 2,
        Some("3") => 3,
        None => {
            error!("No Node ID in json");
            return;
        }
        _ => {
            error!("Invalid Node ID");
            return;
        }
    };
    let dispense_type = json["dispense_type"].as_str().unwrap();
    // placeholder
    let timeout = Duration::from_secs_f64(
        json["timeout"]
            .as_str()
            .and_then(|s| s.parse::<f64>().ok())
            .unwrap(),
    );
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
    let retract_after = json["retract_after"]
        .as_str()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap();
    let parameters = Parameters {
        motor_speed,
        sample_rate,
        cutoff_frequency,
        check_offset,
        stop_offset,
        retract_after: Some(retract_after)
    };
    make_dispenser(
        node_id,
        io.cc2,
        match dispense_type {
            "timed" => {
                info!(
                    "Dispensing Node {:} for {:?}",
                    node_id,
                    json["timeout"].as_str().unwrap()
                );
                Setpoint::Timed(timeout)
            }
            "weight" => {
                info!("Dispensing {:.1} g from Node {:} ", serving_weight, node_id);
                Setpoint::Weight(WeightedDispense {
                    setpoint: serving_weight,
                    timeout,
                })
            }
            "weigh" => {
                let (tx, mut rx) = tokio::sync::oneshot::channel();
                io.scale_txs[node_id].send(ScaleCmd(tx)).await.unwrap();
                match rx.await {
                    Ok(weight) => {
                        info!("Node {:?} weight: {:?} g", node_id, weight)
                    }
                    Err(_) => {
                        warn!("Scale communication failed")
                    }
                }
                return;
            }
            _ => {
                error!("Invalid Dispense Type");
                return;
            }
        },
        parameters,
        io.scale_txs[node_id].clone(),
    )
    .dispense(timeout)
    .await;
    info!("Dispensed from Node {:}", node_id);
}

pub async fn handle_sealer_req(body: Bytes, io: RyoIo) {
    match body[0] {
        b's' => {
            make_sealer(io.clone()).seal().await;
        }
        b'o' => {
            let mut trap_door = make_trap_door(io.clone());
            trap_door.actuate(HBridgeState::Off).await;
            trap_door.actuate(HBridgeState::Neg).await;
            sleep(SEALER_MOVE_DOOR_TIME).await;
            trap_door.actuate(HBridgeState::Off).await;
        }
        b'c' => {
            let mut trap_door = make_trap_door(io.clone());
            trap_door.actuate(HBridgeState::Off).await;
            trap_door.actuate(HBridgeState::Pos).await;
            sleep(SEALER_MOVE_DOOR_TIME).await;
            trap_door.actuate(HBridgeState::Off).await;
        }
        _ => {
            error!("Invalid sealer operation");
        }
    }
}

pub async fn handle_sealer_position_req(body: Bytes, io: RyoIo) {
    match body.len() {
        0 => {
            let pos = make_sealer(io).get_actuator_position().await;
            info!("Sealer at Position {:?}", pos);
        }
        _ => {
            let position = ascii_to_int(body.as_ref());
            info!("Sealer to position {:?}", position);
            let mut sealer = make_sealer(io.clone());
            sealer.absolute_move(position).await;
            let pos = sealer.get_actuator_position().await;
            info!("Sealer at position {:?}", pos);
        }
    }
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
    info!("Cleared Alerts and Enabled All Motors");
}

pub async fn disable_all(io: RyoIo) {
    let cc1_motors: [ClearCoreMotor; 3] = array::from_fn(|motor_id| io.cc1.get_motor(motor_id));
    let cc2_motors: [ClearCoreMotor; 4] = array::from_fn(|motor_id| io.cc2.get_motor(motor_id));

    let disable_cc1_handles = cc1_motors.iter().map(|motor| async move {
        motor.abrupt_stop().await;
        motor.disable().await
    });
    let disable_cc2_handles = cc2_motors.iter().map(|motor| async move {
        motor.abrupt_stop().await;
        motor.disable().await
    });
    join_all(disable_cc1_handles).await;
    join_all(disable_cc2_handles).await;
    info!("Disabled All Motors");
}

// pub async fn response_builder(chunk: &str) -> HTTPResult {
//     Ok(Response::builder()
//         .status(204)
//         .header("Access-Control-Allow-Origin", "*")
//         .header("Access-Control-Allow-Methods", "POST, OPTIONS, GET, PUT")
//         .header("Access-Control-Allow-Headers", "*")
//         .body(full(chunk.to_string()))
//         .unwrap())
// }

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

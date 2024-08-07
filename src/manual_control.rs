use std::time::Duration;
use crate::bag_handler::BagHandler;
use crate::config::{
    GANTRY_ACCELERATION, GANTRY_ALL_POSITIONS, GANTRY_MOTOR_ID, GANTRY_SAMPLE_INTERVAL,
    GANTRY_VELOCITY, SEALER_MOVE_DOOR_TIME, SEALER_MOVE_TIME,
};
use crate::ryo::make_dispenser;
use control_components::components::clear_core_motor::{ClearCoreMotor, Status};
use control_components::components::scale::ScaleCmd;
use control_components::subsystems::dispenser::{Parameters, Setpoint, WeightedDispense};
use log::{error, info, warn};
use tokio::sync::mpsc::Sender;
use crate::CCController;

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

pub async fn enable_and_clear_all(cc: CCController) {
    let motor = cc.get_motor(0);
    motor.clear_alerts().await;
    let _ = motor.enable().await;
    info!("Cleared Alerts and Enabled Motor");
}

pub async fn disable_all(cc: CCController) {
    let motor = cc.get_motor(0);
    motor.clear_alerts().await;
    let _ = motor.enable().await;
    info!("Disabled Motor");
}

pub async fn handle_dispenser_req(json: serde_json::Value, cc: CCController, scale_tx: Sender<ScaleCmd>) {
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
        retract_before: None,
        retract_after: Some(retract_after),
    };
    make_dispenser(
        node_id,
        cc,
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
                let (tx, rx) = tokio::sync::oneshot::channel();
                scale_tx.send(ScaleCmd(tx)).await.unwrap();
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
        scale_tx.clone(),
    )
        .dispense(timeout)
        .await;
    info!("Dispensed from Node {:}", node_id);
}

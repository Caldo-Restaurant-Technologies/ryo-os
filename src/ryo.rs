use control_components::components::clear_core_io::DigitalOutput;
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::components::scale::ScaleCmd;
use control_components::controllers::clear_core::Controller;
use std::array;
use control_components::controllers::{clear_core, ek1100_io};

use crate::config::{BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, CC2_MOTORS, GANTRY_MOTOR_ID, GRIPPER_ACTUATOR, GRIPPER_MOTOR_ID, GRIPPER_POSITIONS, HATCHES_ANALOG_INPUTS, HATCHES_CH_A, HATCHES_CH_B, HATCH_A_CH_A, HATCH_A_CH_B, HATCH_B_CH_A, HATCH_B_CH_B, HATCH_C_CH_A, HATCH_C_CH_B, HATCH_D_CH_A, HATCH_D_CH_B, SEALER_ACTUATOR_ID, SEALER_EXTEND_ID, SEALER_HEATER, SEALER_RETRACT_ID, BAG_DETECT_PE};
// use crate::{CCController, EtherCATIO};
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper, BagSensor};
use control_components::subsystems::dispenser::{Dispenser, Parameters, Setpoint};
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{Output, RelayHBridge, SimpleLinearActuator};
use control_components::subsystems::sealer::Sealer;
use log::error;
use tokio::sync::mpsc::Sender;

type CCController = clear_core::Controller;
type EtherCATIO = ek1100_io::Controller;
#[derive(Clone)]
pub struct RyoIo {
    pub cc1: CCController,
    pub cc2: CCController,
    pub etc_io: EtherCATIO,
    pub scale_txs: [Sender<ScaleCmd>; 4],
}


pub fn make_gripper(cc1: Controller, cc2: Controller) -> BagGripper {
    BagGripper::new(
        cc1.get_motor(GRIPPER_MOTOR_ID),
        SimpleLinearActuator::new(cc2.get_h_bridge(GRIPPER_ACTUATOR)),
        GRIPPER_POSITIONS.to_vec(),
    )
}

pub fn make_bag_dispenser(cc1: Controller) -> BagDispenser {
    BagDispenser::new(
        cc1.get_motor(BAG_ROLLER_MOTOR_ID),
        cc1.get_digital_input(BAG_ROLLER_PE),
    )
}

pub fn make_gantry(controller: CCController) -> ClearCoreMotor {
    controller.get_motor(GANTRY_MOTOR_ID)
}

pub fn make_dispensers(
    cc2: CCController,
    set_points: &[Setpoint],
    parameters: &[Parameters],
    senders: &[Sender<ScaleCmd>],
) -> Vec<Dispenser> {
    (0..4)
        .map(|dispenser_id| {
            Dispenser::new(
                cc2.get_motor(CC2_MOTORS[dispenser_id].id as usize),
                set_points[dispenser_id].clone(),
                parameters[dispenser_id].clone(),
                senders[dispenser_id].clone(),
            )
        })
        .collect()
}

pub fn make_dispenser(
    node_id: usize,
    cc2: CCController,
    set_point: Setpoint,
    parameter: Parameters,
    sender: Sender<ScaleCmd>,
) -> Dispenser {
    Dispenser::new(
        cc2.get_motor(CC2_MOTORS[node_id].id as usize),
        set_point,
        parameter,
        sender,
    )
}

pub fn make_hatches(cc1: Controller, cc2: Controller) -> Vec<Hatch> {
    (0..4)
        .map(|id| make_hatch(id, cc1.clone(), cc2.clone()))
        .collect()
}

pub fn make_hatch(hatch_id: usize, cc1: Controller, cc2: Controller) -> Hatch {
    let cc = match hatch_id {
        0 | 1 => cc1,
        2 | 3 => cc2,
        _ => {
            error!("Invalid Hatch ID");
            cc1
        }
    };
    Hatch::from_io(
        Output::ClearCore(cc.get_output(HATCHES_CH_A[hatch_id])),
        Output::ClearCore(cc.get_output(HATCHES_CH_B[hatch_id])),
        cc.get_analog_input(HATCHES_ANALOG_INPUTS[hatch_id]),
    )
}

pub fn make_sealer(mut io: RyoIo) -> Sealer {
    Sealer::new(
        io.cc1.get_output(SEALER_HEATER),
        io.etc_io.get_io(0),
        3,
        2,
    )
}

pub fn make_trap_door(mut io: RyoIo) -> RelayHBridge {
    RelayHBridge::new(
        (
            Output::EtherCat(io.etc_io.get_io(0), 1, 0),
            Output::EtherCat(io.etc_io.get_io(0), 1, 1),
        ),
        io.cc1.get_analog_input(0),
    )
}

pub fn make_bag_sensor(io: RyoIo) -> BagSensor {
    BagSensor::new(
        io.cc1.get_digital_input(BAG_DETECT_PE)
    )
}
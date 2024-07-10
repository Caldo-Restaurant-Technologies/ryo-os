use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::components::scale::ScaleCmd;
use control_components::controllers::clear_core::Controller;
use std::array;

use crate::config::{
    BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, CC2_MOTORS, GANTRY_MOTOR_ID, GRIPPER_ACTUATOR,
    GRIPPER_MOTOR_ID, GRIPPER_POSITIONS, HATCH_A_CH_A, HATCH_A_CH_B, HATCH_B_CH_A, HATCH_B_CH_B,
    HATCH_C_CH_A, HATCH_C_CH_B, HATCH_D_CH_A, HATCH_D_CH_B,
};
use crate::{CCController, EtherCATIO};
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper};
use control_components::subsystems::dispenser::{Dispenser, Parameters, Setpoint};
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{Output, SimpleLinearActuator};
use tokio::sync::mpsc::Sender;

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
) -> [Dispenser; 4] {
    array::from_fn(|i| {
        Dispenser::new(
            cc2.get_motor(CC2_MOTORS[i].id as usize),
            set_points[i].clone(),
            parameters[i].clone(),
            senders[i].clone(),
        )
    })
}
pub fn make_hatches(cc1: Controller, cc2: Controller) -> [Hatch; 4] {
    [
        Hatch::from_io(
            Output::ClearCore(cc1.get_output(HATCH_A_CH_A)),
            Output::ClearCore(cc1.get_output(HATCH_A_CH_B)),
            cc1.get_analog_input(0),
        ),
        Hatch::from_io(
            Output::ClearCore(cc1.get_output(HATCH_B_CH_A)),
            Output::ClearCore(cc1.get_output(HATCH_B_CH_B)),
            cc1.get_analog_input(0),
        ),
        Hatch::from_io(
            Output::ClearCore(cc2.get_output(HATCH_C_CH_A)),
            Output::ClearCore(cc2.get_output(HATCH_C_CH_B)),
            cc1.get_analog_input(0),
        ),
        Hatch::from_io(
            Output::ClearCore(cc2.get_output(HATCH_D_CH_A)),
            Output::ClearCore(cc2.get_output(HATCH_D_CH_B)),
            cc1.get_analog_input(0),
        ),
    ]
}

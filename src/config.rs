use control_components::components::clear_core_io::DigitalInput;
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::controllers::clear_core::Message;
use tokio::sync::mpsc::Sender;

//E-Stop
pub const E_STOP_INPUT_ID: u8 = 0; // 0 is the same as DI6 in CC, also valid for CC2
pub const E_STOP_OUTPUT_ID: u8 = 0;// I/O 0 in CC
//Motor IDs
pub const GANTRY_MOTOR_ID: u8 = 0;
pub const GRIPPER_MOTOR_ID: u8 = 2;
pub const BAG_ROLLER_MOTOR_ID: u8 = 1;
//Motor IDs CC2
pub const NODE_A_MOTOR_ID: u8 = 0;
pub const NODE_B_MOTOR_ID: u8 = 1;
pub const NODE_C_MOTOR_ID: u8 = 2;
pub const NODE_D_MOTOR_ID: u8 = 3;

// Outputs CC1
pub const SEALER_HEATER: u8 = 1;
pub const BAG_BLOWER: u8 = 5;
pub const SEALER_ACTUATOR_CH_A: u8 = 2;
pub const SEALER_ACTUATOR_CH_B: u8 = 3;
pub const GRIPPER_ACTUATOR: u8 = 4;
pub const SEALER_TRAP_DOOR: u8 = 5;
// Digital Inputs CC1
pub const BAG_ROLLER_PE : u8 = 1;
pub const BAG_DETECT_PE: u8 = 2;
// Analog Inputs CC1
pub const SEALER_TRAP_DOOR_FB: u8 = 3;
pub const SEALER_FB_CH_A: u8 = 4;
pub const SEALER_FB_CH_B: u8 = 5;

//Analog Inputs CC2
pub const NODE_A_FB: u8 = 3;
pub const NODE_B_FB: u8 = 4;
pub const NODE_C_FB: u8 = 5;
pub const NODE_D_FB: u8 = 6;

pub const RYO_MOTOR_COUNT: usize = 7;
pub const RYO_INPUT_COUNT: usize = 3;
pub const CC_STEP_COUNTS: isize = 800;
pub const STEPPER_MOTOR_COUNTS: isize = 200; //TODO: Check hardware counts or change to a common number

pub fn get_motors(
    d1_sender: Sender<Message>, 
    d2_sender: Sender<Message>
) -> [ClearCoreMotor; RYO_MOTOR_COUNT] {
    [
        ClearCoreMotor::new(GANTRY_MOTOR_ID, CC_STEP_COUNTS, d1_sender.clone()),
        ClearCoreMotor::new(BAG_ROLLER_MOTOR_ID, STEPPER_MOTOR_COUNTS, d1_sender.clone()),
        ClearCoreMotor::new(GRIPPER_MOTOR_ID, STEPPER_MOTOR_COUNTS, d1_sender),
        ClearCoreMotor::new(NODE_A_MOTOR_ID, CC_STEP_COUNTS, d2_sender.clone()),
        ClearCoreMotor::new(NODE_B_MOTOR_ID, CC_STEP_COUNTS, d2_sender.clone()),
        ClearCoreMotor::new(NODE_C_MOTOR_ID, CC_STEP_COUNTS, d2_sender.clone()),
        ClearCoreMotor::new(NODE_D_MOTOR_ID, CC_STEP_COUNTS, d2_sender)
    ]
}

pub fn get_inputs(d1_sender: Sender<Message>) -> [DigitalInput; RYO_INPUT_COUNT] {
    [
        DigitalInput::new(E_STOP_INPUT_ID, d1_sender.clone()),
        DigitalInput::new(BAG_ROLLER_PE, d1_sender.clone()),
        DigitalInput::new(BAG_DETECT_PE, d1_sender.clone())
    ]
}

pub struct Gantry {
    drive_id: u8,
    motor_id: u8
}

pub const GANTRY: Gantry = Gantry{
    drive_id:GANTRY_MOTOR_ID,
    motor_id:GANTRY_MOTOR_ID
};

// struct Gripper {
//     drive_id: usize,
//     motor_id: usize,
//     linear_actuator_out: usize,
// }
// 
// struct BagRoller {
//     drive_id: usize,
//     motor_id: usize,
//     photo_eye_id: usize
// }
// 
// struct Hatch {
//     drive_id: usize,
//     motor_id: usize,
// }
// 
// struct Conveyor {
//     drive_id: usize,
//     motor_id: usize,
// }
// struct Node {
//     scale: Scale,
//     conveyor: Conveyor
// }
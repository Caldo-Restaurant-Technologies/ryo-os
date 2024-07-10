
use control_components::controllers::clear_core::MotorBuilder;

//E-Stop
pub const E_STOP_INPUT_ID: usize = 0; // 0 is the same as DI6 in CC, also valid for CC2
pub const E_STOP_OUTPUT_ID: usize = 0; // I/O 0 in CC
                                    //Motor IDs
pub const GANTRY_MOTOR_ID: usize = 0;
pub const GRIPPER_MOTOR_ID: usize = 2;
pub const BAG_ROLLER_MOTOR_ID: usize = 1;
//Motor IDs CC2
pub const NODE_A_MOTOR_ID: usize = 0;
pub const NODE_B_MOTOR_ID: usize = 1;
pub const NODE_C_MOTOR_ID: usize = 2;
pub const NODE_D_MOTOR_ID: usize = 3;

// Outputs CC1
pub const SEALER_HEATER: usize = 1;

pub const HATCH_A_CH_A: usize = 2;
pub const HATCH_A_CH_B: usize = 3;
pub const HATCH_B_CH_A: usize = 4;
pub const HATCH_B_CH_B: usize = 5;
// Outputs CC2
pub const HATCH_C_CH_A: usize = 0;
pub const HATCH_C_CH_B: usize = 1;
pub const HATCH_D_CH_A: usize = 2;
pub const HATCH_D_CH_B: usize = 3;
pub const GRIPPER_ACTUATOR: usize = 4;
pub const BAG_BLOWER: usize = 5;
// Digital Inputs CC1
pub const BAG_ROLLER_PE: usize = 1;
pub const BAG_DETECT_PE: usize = 2;
// Analog Inputs CC1
pub const SEALER_TRAP_DOOR_FB: usize = 3;
pub const SEALER_FB_CH_A: usize = 4;
pub const SEALER_FB_CH_B: usize = 5;

//Analog Inputs CC2
pub const NODE_A_FB: usize = 3;
pub const NODE_B_FB: usize = 4;
pub const NODE_C_FB: usize = 5;
pub const NODE_D_FB: usize = 6;

pub const RYO_MOTOR_COUNT: usize = 7;
pub const RYO_INPUT_COUNT: usize = 3;
pub const CC_STEP_COUNTS: isize = 800;
pub const STEPPER_MOTOR_COUNTS: isize = 200; //TODO: Check hardware counts or change to a common number


pub const CLEAR_CORE_1_ADDR: &str = "192.168.1.11:8888";
pub const CLEAR_CORE_2_ADDR: &str = "192.168.1.12:8888";
pub const LOCAL_INTERFACE: &str = "enp1s0f0";
pub const RYO_INTERFACE: &str = "eth0";

pub const CC1_MOTORS: [MotorBuilder; 3] = [
    MotorBuilder{id: GANTRY_MOTOR_ID as u8, scale: 800},
    MotorBuilder{id: BAG_ROLLER_MOTOR_ID as u8, scale: 200},
    MotorBuilder{id: GRIPPER_MOTOR_ID as u8, scale: 200}
];

pub const CC2_MOTORS: [MotorBuilder; 4] = [
    MotorBuilder{id: NODE_A_MOTOR_ID as u8, scale: 800},
    MotorBuilder{id: NODE_B_MOTOR_ID as u8, scale: 800},
    MotorBuilder{id: NODE_C_MOTOR_ID as u8, scale: 800},
    MotorBuilder{id: NODE_D_MOTOR_ID as u8, scale: 800}
];

pub const PHIDGET_SNS: [i32; 4] = [716709, 716623, 716625, 716620];




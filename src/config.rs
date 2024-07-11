use control_components::controllers::clear_core::MotorBuilder;
use std::time::Duration;

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
pub const CC1_HATCH_IDS: [usize; 4] = [HATCH_A_CH_A, HATCH_A_CH_B, HATCH_B_CH_A, HATCH_B_CH_B];
// Outputs CC2
pub const HATCH_C_CH_A: usize = 0;
pub const HATCH_C_CH_B: usize = 1;
pub const HATCH_D_CH_A: usize = 2;
pub const HATCH_D_CH_B: usize = 3;
pub const CC2_HATCH_IDS: [usize; 4] = [HATCH_C_CH_A, HATCH_C_CH_B, HATCH_D_CH_A, HATCH_D_CH_B];
pub const GRIPPER_ACTUATOR: usize = 4;
pub const BAG_BLOWER: usize = 5;

pub const HATCHES_CH_A: [usize; 4] = [HATCH_A_CH_A, HATCH_B_CH_A, HATCH_C_CH_A, HATCH_D_CH_A];
pub const HATCHES_CH_B: [usize; 4] = [HATCH_A_CH_B, HATCH_B_CH_B, HATCH_C_CH_B, HATCH_D_CH_B];
pub const HATCH_A_ANALOG_INPUT: usize = 0;
pub const HATCH_B_ANALOG_INPUT: usize = 0;
pub const HATCH_C_ANALOG_INPUT: usize = 0;
pub const HATCH_D_ANALOG_INPUT: usize = 0;
pub const HATCHES_ANALOG_INPUTS: [usize; 4] = [
    HATCH_A_ANALOG_INPUT,
    HATCH_B_ANALOG_INPUT,
    HATCH_C_ANALOG_INPUT,
    HATCH_D_ANALOG_INPUT,
];

pub const HATCHES_OPEN_TIME: Duration = Duration::from_millis(1900);
pub const HATCH_A_CLOSE_TIME: Duration = Duration::from_millis(1550);
pub const HATCH_B_CLOSE_TIME: Duration = Duration::from_millis(1550);
pub const HATCH_C_CLOSE_TIME: Duration = Duration::from_millis(1500);
pub const HATCH_D_CLOSE_TIME: Duration = Duration::from_millis(1550);
pub const HATCH_CLOSE_TIMES: [Duration; 4] = [
    HATCH_A_CLOSE_TIME,
    HATCH_B_CLOSE_TIME,
    HATCH_C_CLOSE_TIME,
    HATCH_D_CLOSE_TIME,
];

// Digital Inputs CC1
pub const BAG_ROLLER_PE: usize = 1;
pub const BAG_DETECT_PE: usize = 2;
// Analog Inputs CC1
pub const SEALER_TRAP_DOOR_FB: usize = 3;
pub const SEALER_EXTEND_ID: u8 = 0;
pub const SEALER_RETRACT_ID: u8 = 1;
pub const SEALER_ACTUATOR_ID: usize = 1;

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
    MotorBuilder {
        id: GANTRY_MOTOR_ID as u8,
        scale: 800,
    },
    MotorBuilder {
        id: BAG_ROLLER_MOTOR_ID as u8,
        scale: 200,
    },
    MotorBuilder {
        id: GRIPPER_MOTOR_ID as u8,
        scale: 200,
    },
];

pub const CC2_MOTORS: [MotorBuilder; 4] = [
    MotorBuilder {
        id: NODE_A_MOTOR_ID as u8,
        scale: 800,
    },
    MotorBuilder {
        id: NODE_B_MOTOR_ID as u8,
        scale: 800,
    },
    MotorBuilder {
        id: NODE_C_MOTOR_ID as u8,
        scale: 800,
    },
    MotorBuilder {
        id: NODE_D_MOTOR_ID as u8,
        scale: 800,
    },
];

pub const PHIDGET_SNS: [i32; 4] = [716709, 716623, 716625, 716620];

pub const GRIPPER_POSITIONS: [f64; 3] = [-0.4, 0.8, 0.4];

pub const DISPENSER_TIMEOUT: Duration = Duration::from_secs(120);

pub const GANTRY_NODE_POSITIONS: [f64; 4] = [24.5, 47.0, 69.5, 92.0];
pub const GANTRY_HOME_POSITION: f64 = 0.;
pub const GANTRY_BAG_DROP_POSITION: f64 = 84.;
pub const GANTRY_ALL_POSITIONS: [f64; 6] = [
    GANTRY_HOME_POSITION,
    GANTRY_NODE_POSITIONS[0],
    GANTRY_NODE_POSITIONS[1],
    GANTRY_NODE_POSITIONS[2],
    GANTRY_NODE_POSITIONS[3],
    GANTRY_BAG_DROP_POSITION,
];
pub const GANTRY_SAMPLE_INTERVAL: Duration = Duration::from_millis(250);

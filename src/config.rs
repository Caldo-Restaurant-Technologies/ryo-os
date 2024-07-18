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

pub const ETHERCAT_RACK_ID: usize = 0;
pub const HATCHES_SLOT_ID: usize = 1;
pub const HATCHES_OPEN_OUTPUT_IDS: [usize; 4] = [0, 2, 4, 6];
pub const HATCHES_CLOSE_OUTPUT_IDS: [usize; 4] = [1, 3, 5, 7];
pub const HATCHES_ANALOG_INPUTS: [usize; 4] = [0, 1, 2, 3];
pub const HATCHES_OPEN_SET_POINTS: [isize; 4] = [500, 500, 500, 500];
pub const HATCHES_CLOSE_SET_POINTS: [isize; 4] = [3900, 3900, 3900, 3900];

pub const SEALER_SLOT_ID: usize = 2;
pub const SEALER_EXTEND_OUTPUT_ID: usize = 1;
pub const SEALER_RETRACT_OUTPUT_ID: usize = 0;
pub const SEALER_ANALOG_INPUT: usize = 0; // also 1 since there are two actuators
pub const SEALER_TIMEOUT: Duration = Duration::from_secs(10);
pub const SEALER_EXTEND_SET_POINT: isize = 500;
pub const SEALER_RETRACT_SET_POINT: isize = 3900;
pub const TRAP_DOOR_SLOT_ID: usize = 2;
pub const TRAP_DOOR_OPEN_OUTPUT_ID: usize = 3;
pub const TRAP_DOOR_CLOSE_OUTPUT_ID: usize = 2;

pub const HEATER_SLOT_ID: usize = 3;
pub const HEATER_OUTPUT_ID: usize = 2;
pub const BLOWER_SLOT_ID: usize = 3;
pub const BLOWER_OUTPUT_ID: usize = 3;

pub const GRIPPER_ACTUATOR: usize = 5;
pub const BAG_BLOWER: usize = 5;

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
pub const SEALER_EXTEND_ID: u8 = 1;
pub const SEALER_RETRACT_ID: u8 = 0;
pub const SEALER_ACTUATOR_ID: usize = 1;
pub const SEALER_MOVE_DOOR_TIME: Duration = Duration::from_millis(2700);

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

pub const GANTRY_NODE_POSITIONS: [f64; 4] = [26.5, 49., 71.5, 94.];
pub const GANTRY_HOME_POSITION: f64 = 0.;
pub const GANTRY_BAG_DROP_POSITION: f64 = 82.;
pub const GANTRY_ALL_POSITIONS: [f64; 6] = [
    GANTRY_HOME_POSITION,
    GANTRY_NODE_POSITIONS[0],
    GANTRY_NODE_POSITIONS[1],
    GANTRY_NODE_POSITIONS[2],
    GANTRY_NODE_POSITIONS[3],
    GANTRY_BAG_DROP_POSITION,
];
pub const GANTRY_SAMPLE_INTERVAL: Duration = Duration::from_millis(250);
pub const ETHERCAT_NUMBER_OF_SLOTS: u8 = 3;

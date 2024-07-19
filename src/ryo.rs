use control_components::components::clear_core_io::{HBridgeState};
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::components::scale::ScaleCmd;
use control_components::controllers::clear_core::Controller;
use control_components::controllers::{clear_core, ek1100_io};
use std::array;
use std::time::Duration;

use crate::bag_handler::BagHandler;
use crate::config::{
    BAG_DETECT_PE, BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, CC2_MOTORS, DISPENSER_TIMEOUT,
    ETHERCAT_RACK_ID, GANTRY_BAG_DROP_POSITION, GANTRY_HOME_POSITION, GANTRY_MOTOR_ID,
    GANTRY_NODE_POSITIONS, GANTRY_SAMPLE_INTERVAL, GRIPPER_MOTOR_ID,
    GRIPPER_POSITIONS, HATCHES_ANALOG_INPUTS, HATCHES_CLOSE_OUTPUT_IDS, HATCHES_CLOSE_SET_POINTS,
    HATCHES_OPEN_OUTPUT_IDS, HATCHES_OPEN_SET_POINTS, HATCHES_OPEN_TIME, HATCHES_SLOT_ID,
    HATCH_CLOSE_TIMES, HEATER_OUTPUT_ID, HEATER_SLOT_ID, SEALER_ACTUATOR_ID, SEALER_ANALOG_INPUT,
    SEALER_EXTEND_ID, SEALER_EXTEND_SET_POINT, SEALER_HEATER, SEALER_MOVE_DOOR_TIME,
    SEALER_RETRACT_ID, SEALER_RETRACT_SET_POINT, SEALER_SLOT_ID, SEALER_TIMEOUT,
    TRAP_DOOR_CLOSE_OUTPUT_ID, TRAP_DOOR_OPEN_OUTPUT_ID, TRAP_DOOR_SLOT_ID,
};
use control_components::subsystems::bag_handling::{
    BagDispenser, BagSensor,
};
use control_components::subsystems::dispenser::{Dispenser, Parameters, Setpoint};
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{Output, RelayHBridge, SimpleLinearActuator};
use control_components::subsystems::sealer::Sealer;
use futures::future::join_all;
use log::{info};
use tokio::sync::mpsc::Sender;
use tokio::task::JoinHandle;
use tokio::time::sleep;

type CCController = clear_core::Controller;
type EtherCATIO = ek1100_io::Controller;
#[derive(Clone)]
pub struct RyoIo {
    pub cc1: CCController,
    pub cc2: CCController,
    pub etc_io: EtherCATIO,
    pub scale_txs: [Sender<ScaleCmd>; 4],
}

#[derive(Debug, Clone)]
pub enum BagLoadedState {
    Bagful,
    Bagless,
}

#[derive(Debug, Clone)]
pub enum NodeState {
    Ready,
    Dispensed,
}

#[derive(Debug, Clone)]
pub enum BagFilledState {
    Empty,
    Filling,
    Filled,
}

#[derive(Debug, Clone)]
pub struct RyoState {
    bag_loaded: BagLoadedState,
    nodes: [NodeState; 4],
    bag_filled: Option<BagFilledState>,
}
impl RyoState {
    pub fn fresh() -> Self {
        Self {
            bag_loaded: BagLoadedState::Bagless,
            nodes: array::from_fn(|_| NodeState::Ready),
            bag_filled: None,
        }
    }

    pub fn set_bag_filled_state(&mut self, state: Option<BagFilledState>) {
        self.bag_filled = state;
    }

    pub fn set_node_state(&mut self, id: usize, state: NodeState) {
        self.nodes[id] = state;
    }

    pub fn set_bag_loaded_state(&mut self, state: BagLoadedState) {
        self.bag_loaded = state;
    }

    pub fn set_all_node_states(&mut self, state: NodeState) {
        for i in 0..self.nodes.len() {
            self.set_node_state(i, state.clone())
        }
    }

    pub fn get_node_state(&self, id: usize) -> NodeState {
        self.nodes[id].clone()
    }

    pub fn get_bag_filled_state(&self) -> Option<BagFilledState> {
        self.bag_filled.clone()
    }

    pub fn get_bag_loaded_state(&self) -> BagLoadedState {
        self.bag_loaded.clone()
    }
}

pub enum RyoFailure {
    BagFailure,
    NodeFailure,
}

pub fn make_bag_handler(io: RyoIo) -> BagHandler {
    BagHandler::new(io)
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
    ids: Vec<usize>,
    cc2: CCController,
    set_points: &[Setpoint],
    parameters: &[Parameters],
    senders: &[Sender<ScaleCmd>],
) -> Vec<Dispenser> {
    ids.into_iter()
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

pub fn make_hatches(io: RyoIo) -> Vec<Hatch> {
    (0..4).map(|id| make_hatch(id, io.clone())).collect()
}

pub fn make_hatch(hatch_id: usize, mut io: RyoIo) -> Hatch {
    Hatch::from_io(
        Output::EtherCat(
            io.etc_io.get_io(ETHERCAT_RACK_ID),
            HATCHES_SLOT_ID,
            HATCHES_OPEN_OUTPUT_IDS[hatch_id] as u8,
        ),
        Output::EtherCat(
            io.etc_io.get_io(ETHERCAT_RACK_ID),
            HATCHES_SLOT_ID,
            HATCHES_CLOSE_OUTPUT_IDS[hatch_id] as u8,
        ),
        io.cc2.get_analog_input(HATCHES_ANALOG_INPUTS[hatch_id]),
    )
}

pub async fn make_and_move_hatch(hatch_id: usize, position: isize, io: RyoIo) {
    let mut hatch = make_hatch(hatch_id, io);
    if hatch.get_position().await > position {
        hatch.open(position).await
    } else {
        hatch.close(position).await
    }
}

pub async fn make_and_open_hatch(hatch_id: usize, io: RyoIo) {
    make_and_move_hatch(hatch_id, HATCHES_OPEN_SET_POINTS[hatch_id], io).await
}

pub async fn make_and_close_hatch(hatch_id: usize, io: RyoIo) {
    make_and_move_hatch(hatch_id, HATCHES_CLOSE_SET_POINTS[hatch_id], io).await
}

pub fn make_sealer(mut io: RyoIo) -> Sealer {
    Sealer::new(
        Output::EtherCat(
            io.etc_io.get_io(ETHERCAT_RACK_ID),
            HEATER_SLOT_ID,
            HEATER_OUTPUT_ID as u8,
        ),
        RelayHBridge::new(
            (
                Output::EtherCat(
                    io.etc_io.get_io(ETHERCAT_RACK_ID),
                    SEALER_SLOT_ID,
                    SEALER_EXTEND_ID,
                ),
                Output::EtherCat(
                    io.etc_io.get_io(ETHERCAT_RACK_ID),
                    SEALER_SLOT_ID,
                    SEALER_RETRACT_ID,
                ),
            ),
            io.cc1.get_analog_input(SEALER_ANALOG_INPUT),
        ),
        SEALER_TIMEOUT,
        SEALER_EXTEND_SET_POINT,
        SEALER_RETRACT_SET_POINT,
    )
}

pub fn make_trap_door(mut io: RyoIo) -> RelayHBridge {
    // Hatch::from_io(
    //     Output::EtherCat(
    //         io.etc_io.get_io(ETHERCAT_RACK_ID),
    //
    //     )
    // )

    RelayHBridge::new(
        (
            Output::EtherCat(
                io.etc_io.get_io(ETHERCAT_RACK_ID),
                TRAP_DOOR_SLOT_ID,
                TRAP_DOOR_OPEN_OUTPUT_ID as u8,
            ),
            Output::EtherCat(
                io.etc_io.get_io(ETHERCAT_RACK_ID),
                TRAP_DOOR_SLOT_ID,
                TRAP_DOOR_CLOSE_OUTPUT_ID as u8,
            ),
        ),
        io.cc1.get_analog_input(0),
    )
}

pub fn make_bag_sensor(io: RyoIo) -> BagSensor {
    BagSensor::new(io.cc1.get_digital_input(BAG_DETECT_PE))
}

pub fn make_default_dispense_tasks(ids: Vec<usize>, io: RyoIo) -> Vec<JoinHandle<()>> {
    let mut dispensers = Vec::with_capacity(4);
    for id in ids {
        let params = Parameters::default();
        let set_point = Setpoint::Timed(Duration::from_secs(10));
        dispensers.push(make_dispenser(
            id,
            io.cc2.clone(),
            set_point,
            params,
            io.scale_txs[id].clone(),
        ))
    }

    dispensers
        .into_iter()
        .map(|dispenser| tokio::spawn(async move { dispenser.dispense(DISPENSER_TIMEOUT).await }))
        .collect()
}

pub fn make_bag_load_task(io: RyoIo) -> JoinHandle<()> {
    let mut bag_handler = BagHandler::new(io);
    tokio::spawn(async move { bag_handler.load_bag().await })
}

pub async fn dump_from_hatch(id: usize, io: RyoIo) {
    let mut hatch = make_hatch(id, io.clone());
    hatch.timed_open(HATCHES_OPEN_TIME).await;
    sleep(Duration::from_millis(500)).await;
    hatch.timed_close(HATCH_CLOSE_TIMES[id]).await;
}

pub async fn drop_bag(io: RyoIo) {
    let gantry = make_gantry(io.cc1.clone());
    let _ = gantry.absolute_move(GANTRY_BAG_DROP_POSITION).await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
    // let mut gripper = make_gripper(io.cc1.clone(), io.cc2.clone());
    let mut bag_handler = make_bag_handler(io);
    bag_handler.open_gripper().await;
    sleep(Duration::from_millis(500)).await;
    bag_handler.close_gripper().await;

    let _ = gantry.absolute_move(GANTRY_NODE_POSITIONS[2]).await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
}

pub async fn release_bag_from_sealer(io: RyoIo) {
    let mut trap_door = make_trap_door(io.clone());
    trap_door.actuate(HBridgeState::Neg).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    trap_door.actuate(HBridgeState::Off).await;
    sleep(Duration::from_millis(500)).await;
    trap_door.actuate(HBridgeState::Pos).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    trap_door.actuate(HBridgeState::Off).await;
}

pub async fn pull_after_flight(io: RyoIo) {
    let gantry = make_gantry(io.cc1.clone());
    let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
    BagHandler::new(io.clone()).dispense_bag().await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
}

pub async fn set_motor_accelerations(io: RyoIo, acceleration: f64) {
    let cc1_motors: [ClearCoreMotor; 1] = array::from_fn(|motor_id| io.cc1.get_motor(motor_id));
    let cc2_motors: [ClearCoreMotor; 4] = array::from_fn(|motor_id| io.cc2.get_motor(motor_id));

    let enable_clear_cc1_handles = cc1_motors.iter().map(|motor| async move {
        motor.clear_alerts().await;
        let _ = motor.enable().await;
        motor.set_acceleration(acceleration).await;
    });
    let enable_clear_cc2_handles = cc2_motors.iter().map(|motor| async move {
        motor.clear_alerts().await;
        let _ = motor.enable().await;
        motor.set_acceleration(acceleration).await;
    });
    join_all(enable_clear_cc1_handles).await;
    join_all(enable_clear_cc2_handles).await;
    info!("Cleared Alerts and Enabled All Motors");
}

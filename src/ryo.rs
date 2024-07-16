use control_components::components::clear_core_io::{DigitalOutput, HBridgeState};
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::components::scale::ScaleCmd;
use control_components::controllers::clear_core::Controller;
use std::array;
use std::time::Duration;
use control_components::controllers::{clear_core, ek1100_io};

use crate::config::{BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, CC2_MOTORS, GANTRY_MOTOR_ID, GRIPPER_ACTUATOR, GRIPPER_MOTOR_ID, GRIPPER_POSITIONS, HATCHES_ANALOG_INPUTS, HATCHES_CH_A, HATCHES_CH_B, HATCH_A_CH_A, HATCH_A_CH_B, HATCH_B_CH_A, HATCH_B_CH_B, HATCH_C_CH_A, HATCH_C_CH_B, HATCH_D_CH_A, HATCH_D_CH_B, SEALER_ACTUATOR_ID, SEALER_EXTEND_ID, SEALER_HEATER, SEALER_RETRACT_ID, BAG_DETECT_PE, DISPENSER_TIMEOUT, GANTRY_NODE_POSITIONS, GANTRY_SAMPLE_INTERVAL, HATCHES_OPEN_TIME, HATCH_CLOSE_TIMES, GANTRY_BAG_DROP_POSITION, SEALER_MOVE_DOOR_TIME, GANTRY_HOME_POSITION};
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper, BagSensor, BagSensorState};
use control_components::subsystems::dispenser::{Dispenser, Parameters, Setpoint};
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{Output, RelayHBridge, SimpleLinearActuator};
use control_components::subsystems::sealer::Sealer;
use log::{error, info};
use tokio::sync::mpsc::Sender;
use tokio::task::JoinHandle;
use tokio::time::sleep;
use crate::bag_handler::BagHandler;


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
pub enum BagState {
    Bagful,
    Bagless,
}

#[derive(Debug, Clone)]
pub enum NodeState {
    Ready,
    Dispensed,
}

#[derive(Debug, Clone)]
pub struct RyoState {
    bag: BagState,
    nodes: [NodeState; 4],
} impl RyoState {
    pub fn fresh() -> Self {
        Self {
            bag: BagState::Bagless,
            nodes: array::from_fn(|_| NodeState::Ready)
        }
    }

    pub fn set_bag_state(&mut self, state: BagState) {
        self.bag = state;
    }

    pub fn set_node_state(&mut self, id: usize, state: NodeState) {
        self.nodes[id] = state;
    }

    pub fn set_all_node_states(&mut self, state: NodeState) {
        for i in 0..self.nodes.len() {
            self.set_node_state(i, state.clone())
        }
    }

    pub fn get_node_state(&self, id: usize) -> NodeState {
        self.nodes[id].clone()
    }
    
    pub fn get_bag_state(&self) -> BagState {
        self.bag.clone()
    }
    
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
    ids: Vec<usize>,
    cc2: CCController,
    set_points: &[Setpoint],
    parameters: &[Parameters],
    senders: &[Sender<ScaleCmd>],
) -> Vec<Dispenser> {
    ids
        .into_iter().map(|dispenser_id| {
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

pub fn make_default_dispense_tasks(ids: Vec<usize>, io: RyoIo) -> Vec<JoinHandle<()>> {
    let mut dispensers = Vec::new();
    for id in ids {
        let params = Parameters::default();
        let set_point = Setpoint::Timed(Duration::from_secs(15));
        dispensers.push(
            make_dispenser(id, io.cc2.clone(), set_point, params, io.scale_txs[id].clone())
        )
    }
    
    dispensers
        .into_iter()
        .map(|dispenser| {
            tokio::spawn(async move { dispenser.dispense(DISPENSER_TIMEOUT).await })
        })
        .collect()
}

pub fn make_bag_load_task(io: RyoIo) -> JoinHandle<()> {
    let mut bag_handler = BagHandler::new(io.cc1, io.cc2);
    tokio::spawn(async move { bag_handler.load_bag().await })
}

pub async fn dump_from_hatch(id: usize, io: RyoIo) {
    let gantry = make_gantry(io.cc1.clone());
    let bag_sensor = make_bag_sensor(io.clone()); 
    let mut hatch = make_hatch(id, io.cc1.clone(), io.cc2.clone());
    
    let _ = gantry
        .absolute_move(GANTRY_NODE_POSITIONS[id])
        .await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
    hatch.timed_open(HATCHES_OPEN_TIME).await;
    sleep(Duration::from_millis(500)).await;
    hatch.timed_close(HATCH_CLOSE_TIMES[id]).await;
}

pub async fn drop_bag(io: RyoIo) {
    let gantry = make_gantry(io.cc1.clone());
    let _ = gantry
        .absolute_move(GANTRY_BAG_DROP_POSITION)
        .await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
    let mut gripper = make_gripper(io.cc1.clone(), io.cc2.clone());
    gripper.open().await;
    sleep(Duration::from_millis(500)).await;
    gripper.close().await;
    
    let _ = gantry.absolute_move(GANTRY_NODE_POSITIONS[2]).await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
}

pub async fn release_bag_from_sealer(io: RyoIo) {
    let mut trap_door =make_trap_door(io.clone());
    trap_door.actuate(HBridgeState::Neg).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    trap_door.actuate(HBridgeState::Off).await;
    sleep(Duration::from_millis(500)).await;
    trap_door.actuate(HBridgeState::Pos).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    trap_door.actuate(HBridgeState::Off).await;
}

pub async fn reset_for_next_cycle(io: RyoIo) {
    let gantry = make_gantry(io.cc1.clone());
    let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
    BagHandler::new(io.cc1.clone(), io.cc2.clone()).dispense_bag().await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await;
}

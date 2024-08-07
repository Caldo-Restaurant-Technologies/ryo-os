use control_components::components::clear_core_io::HBridgeState;
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::components::scale::ScaleCmd;
use control_components::controllers::clear_core::Controller;
use control_components::controllers::{clear_core, ek1100_io};
use std::array;
use std::error::Error;
use std::time::Duration;

use crate::app_integration::JobOrder;
use crate::bag_handler::BagHandler;
use crate::config::{
    BAG_DETECT_PE, BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, CC2_MOTORS, DEFAULT_DISPENSE_PARAMETERS,
    DISPENSER_TIMEOUT, ETHERCAT_RACK_ID, GANTRY_ACCELERATION, GANTRY_BAG_DROP_POSITION,
    GANTRY_HOME_POSITION, GANTRY_MOTOR_ID, GANTRY_NODE_POSITIONS, GANTRY_SAMPLE_INTERVAL,
    HATCHES_ANALOG_INPUTS, HATCHES_CLOSE_OUTPUT_IDS, HATCHES_CLOSE_SET_POINTS,
    HATCHES_OPEN_OUTPUT_IDS, HATCHES_OPEN_SET_POINTS, HATCHES_SLOT_ID, HATCH_TIMEOUT,
    HEATER_OUTPUT_ID, HEATER_SLOT_ID, NODE_LOW_THRESHOLDS, NUMBER_OF_NODES, POTATO_HASH_RECIPE,
    SEALER_ANALOG_INPUT, SEALER_EXTEND_ID, SEALER_EXTEND_SET_POINT, SEALER_MOVE_DOOR_TIME,
    SEALER_RETRACT_ID, SEALER_RETRACT_SET_POINT, SEALER_SLOT_ID, SEALER_TIMEOUT,
    TRAP_DOOR_CLOSE_OUTPUT_ID, TRAP_DOOR_OPEN_OUTPUT_ID, TRAP_DOOR_SLOT_ID,
};
use crate::manual_control::enable_and_clear_all;
use crate::sealer::SealerCmd;
use control_components::subsystems::bag_handling::{BagDispenser, BagSensor, BagSensorState};
use control_components::subsystems::dispenser::{
    DispenseParameters, Dispenser, Parameters, Setpoint,
};
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{Output, RelayHBridge};
use control_components::subsystems::sealer::Sealer;
use futures::future::join_all;
use log::{error, info};
use tokio::sync::mpsc::Sender;
use tokio::task::{JoinHandle, JoinSet};
use tokio::time::sleep;

type CCController = clear_core::Controller;
type EtherCATIO = ek1100_io::Controller;
#[derive(Clone)]
pub struct RyoIo {
    pub cc1: CCController,
    pub cc2: CCController,
    pub etc_io: EtherCATIO,
    pub scale_txs: [Sender<ScaleCmd>; 4],
    pub sealer_tx: Sender<SealerCmd>,
}

#[derive(Debug, Clone)]
pub enum BagState {
    Bagful(BagFilledState),
    Bagless,
}

#[derive(Debug, Clone)]
pub enum NodeState {
    Ready,
    Dispensed,
    Empty,
}

#[derive(Debug, Clone)]
pub enum BagFilledState {
    Empty,
    Filling,
    Filled,
}

#[derive(Debug, Clone)]
pub enum RyoRunState {
    Ready,
    Faulted,
    Running,
    UI,
    NewJob,
}

#[derive(Debug, Clone)]
pub struct RyoRecipe {}

#[derive(Debug, Clone)]
pub struct RyoState {
    bag: BagState,
    nodes: [NodeState; 4],
    failures: Vec<RyoFailure>,
    run_state: RyoRunState,
    is_single_ingredient: bool,
    recipe: [Option<DispenseParameters>; 4],
    // jobs_remaining: usize,
}
impl Default for RyoState {
    fn default() -> Self {
        Self {
            bag: BagState::Bagless,
            nodes: array::from_fn(|_| NodeState::Ready),
            failures: Vec::new(),
            // run_state: RyoRunState::Ready,
            // TODO: put this on new job to start
            run_state: RyoRunState::UI,
            is_single_ingredient: true,
            recipe: [
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
            ],
            // jobs_remaining: 0,
        }
    }
}
impl RyoState {
    pub fn new() -> Self {
        Self {
            bag: BagState::Bagless,
            nodes: array::from_fn(|_| NodeState::Ready),
            failures: Vec::new(),
            run_state: RyoRunState::Ready,
            is_single_ingredient: true,
            recipe: [
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
            ],
        }
    }
    pub fn new_with_recipe(recipe: [Option<DispenseParameters>; 4]) -> Self {
        Self {
            bag: BagState::Bagless,
            nodes: array::from_fn(|_| NodeState::Ready),
            failures: Vec::new(),
            run_state: RyoRunState::Ready,
            is_single_ingredient: false,
            recipe,
        }
    }

    pub async fn update_node_levels(&mut self, ryo_io: RyoIo) {
        let mut weights = Vec::with_capacity(4);
        for scale_tx in ryo_io.scale_txs {
            let (tx, rx) = tokio::sync::oneshot::channel();
            scale_tx
                .send(ScaleCmd(tx))
                .await
                .expect("Failed to send ScaleCmd");
            let weight = rx.await.expect("Failed to unwrap weight");
            weights.push(weight);
        }
        for (id, weight) in weights.iter().enumerate() {
            if *weight <= NODE_LOW_THRESHOLDS[id] {
                info!("Node {:?} Empty", id);
                self.nodes[id] = NodeState::Empty;
            }
        }
        for node_state in self.nodes.clone() {
            match node_state {
                NodeState::Empty => continue,
                _ => return,
            }
        }
        self.set_run_state(RyoRunState::Faulted);
    }

    pub fn set_run_state(&mut self, state: RyoRunState) {
        self.run_state = state;
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

    pub fn set_dispenser_recipe(&mut self, id: usize, recipe: DispenseParameters) {
        self.recipe[id] = Some(recipe);
    }

    pub fn set_recipe(&mut self, job_order: &JobOrder) {
        let ingredients = job_order.get_ingredients();
        self.recipe = ingredients.map(|ingredient| match ingredient {
            Some(ing) => Some(ing.get_parameters()),
            None => None,
        });
    }

    pub fn set_is_single_ingredient(&mut self, is_single_ingredient: bool) {
        self.is_single_ingredient = is_single_ingredient
    }

    pub fn get_run_state(&self) -> RyoRunState {
        self.run_state.clone()
    }
    pub fn get_node_state(&self, id: usize) -> NodeState {
        self.nodes[id].clone()
    }

    pub fn get_bag_state(&self) -> BagState {
        self.bag.clone()
    }

    pub fn get_failures(&self) -> Vec<RyoFailure> {
        self.failures.clone()
    }

    pub fn get_dispenser_recipe(&self, id: usize) -> Option<DispenseParameters> {
        self.recipe[id].clone()
    }

    pub fn get_recipe(&self) -> [Option<DispenseParameters>; 4] {
        self.recipe.clone()
    }

    pub fn get_is_single_ingredient(&self) -> bool {
        self.is_single_ingredient
    }

    pub fn get_first_available_ingredient_id(&self) -> Option<usize> {
        for (id, ingredient) in self.recipe.iter().enumerate() {
            if ingredient.is_some() {
                match self.get_node_state(id) {
                    NodeState::Ready | NodeState::Dispensed => return Some(id),
                    NodeState::Empty => (),
                }
            }
        }
        None
    }

    pub fn log_failure(&mut self, failure: RyoFailure) {
        self.failures.push(failure);
        if self.failures.len() > 5 {
            self.failures.reverse();
            self.failures.pop();
            self.failures.reverse();
        }
    }

    pub fn clear_failures(&mut self) {
        self.failures = Vec::new();
    }

    pub fn check_failures(&mut self) {
        match self.get_run_state() {
            RyoRunState::Faulted | RyoRunState::UI => (),
            RyoRunState::Ready | RyoRunState::Running | RyoRunState::NewJob => {
                if self.failures.len() > 3 {
                    let first_failure = &self.failures[0];
                    let all_same = self.failures.iter().all(|f| f == first_failure);
                    if all_same {
                        error!("Repeated Failure: {:?}", first_failure);
                        error!("Resolve Failure and Update to Continue...");
                        self.set_run_state(RyoRunState::Faulted);
                    }
                }
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum RyoFailure {
    BagDispenseFailure,
    NodeFailure,
    BagDroppingFailure,
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

pub async fn make_gantry(controller: CCController) -> ClearCoreMotor {
    let gantry = controller.get_motor(GANTRY_MOTOR_ID);
    let _ = gantry.enable().await;
    gantry.set_acceleration(GANTRY_ACCELERATION).await;
    gantry.set_deceleration(GANTRY_ACCELERATION).await;
    gantry
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

// pub fn make_dispenser_from_ingredient(
//     node_id: usize,
//     ingredient: Ingredient,
//     io: RyoIo,
// ) -> Dispenser {
//     Dispenser::new(
//         io.cc2.get_motor(node_id),
//         Setpoint::Weight(WeightedDispense {
//             setpoint: ingredient.get_portion_size(),
//             timeout: DEFAULT_DISPENSER_TIMEOUT,
//         }),
//         ingredient.get_parameters(),
//         io.scale_txs[node_id].clone(),
//     )
// }

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
        HATCH_TIMEOUT,
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

pub fn make_dispense_tasks(mut state: RyoState, io: RyoIo) -> (RyoState, Vec<JoinHandle<()>>) {
    let mut dispensers = Vec::with_capacity(4);
    info!("IsSingleIngredient: {:?}", state.get_is_single_ingredient());
    match state.get_is_single_ingredient() {
        false => {
            for (id, subrecipe) in state.get_recipe().iter().enumerate() {
                if let Some(sub) = subrecipe {
                    match state.get_node_state(id) {
                        NodeState::Dispensed => (),
                        NodeState::Ready => {
                            let params = sub.clone().parameters;
                            let setpoint = sub.clone().setpoint;
                            dispensers.push(make_dispenser(
                                id,
                                io.cc2.clone(),
                                setpoint,
                                params,
                                io.scale_txs[id].clone(),
                            ));
                            state.set_node_state(id, NodeState::Dispensed);
                        }
                        NodeState::Empty => {
                            error!("Node {:?} is empty", id);
                            state.set_run_state(RyoRunState::Faulted);
                            return (state, Vec::new());
                        }
                    }
                }
            }
        }
        true => {
            match state.get_first_available_ingredient_id() {
                Some(id) => {
                    match state.get_node_state(id) {
                        NodeState::Dispensed => (),
                        NodeState::Ready => {
                            info!("Bag not yet filled, dispensing");
                            let params = state.get_recipe()[id].clone().unwrap().parameters;
                            let setpoint = state.get_recipe()[id].clone().unwrap().setpoint;
                            dispensers.push(make_dispenser(
                                id,
                                io.cc2.clone(),
                                setpoint,
                                params,
                                io.scale_txs[id].clone(),
                            ));
                            state.set_node_state(id, NodeState::Dispensed);
                        }
                        NodeState::Empty => {
                            state.set_run_state(RyoRunState::Faulted);
                            return (state, Vec::new());
                        } // TODO: this shouldn't ever be encountered?
                    }
                }
                None => {
                    error!("No loaded nodes!");
                    state.set_run_state(RyoRunState::Faulted);
                    return (state, Vec::new());
                }
            }
        }
    }

    // (
    //     state,
    //     dispensers
    //         .into_iter()
    //         .map(|dispenser| {
    //             tokio::spawn(async move { dispenser.dispense(DISPENSER_TIMEOUT).await })
    //         })
    //         .collect(),
    // )

    let mut handles = Vec::with_capacity(NUMBER_OF_NODES);
    for dispenser in dispensers {
        handles.push(tokio::spawn(async move {
            dispenser.dispense(DISPENSER_TIMEOUT).await;
        }))
    }

    (state, handles)
}

pub fn make_bag_load_task(io: RyoIo) -> JoinHandle<()> {
    let mut bag_handler = BagHandler::new(io);
    tokio::spawn(async move { bag_handler.load_bag().await })
}

pub async fn dump_from_hatch(id: usize, io: RyoIo) {
    let mut hatch = make_hatch(id, io);
    hatch.open(HATCHES_OPEN_SET_POINTS[id]).await;
    sleep(Duration::from_millis(100)).await;
    tokio::spawn(async move {
        hatch.close(HATCHES_CLOSE_SET_POINTS[id]).await;
    });
}
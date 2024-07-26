use control_components::components::clear_core_io::HBridgeState;
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::components::scale::ScaleCmd;
use control_components::controllers::clear_core::Controller;
use control_components::controllers::{clear_core, ek1100_io};
use std::io::Write;
use std::sync::Arc;
use std::time::Duration;
use std::{array, io};

use crate::app_integration::{JobOrder, Status};
use crate::bag_handler::BagHandler;
use crate::config::{
    BAG_DETECT_PE, BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, CC2_MOTORS, DEFAULT_DISPENSER_TIMEOUT,
    DEFAULT_DISPENSE_PARAMETERS, DISPENSER_TIMEOUT, ETHERCAT_RACK_ID, GANTRY_ACCELERATION,
    GANTRY_BAG_DROP_POSITION, GANTRY_HOME_POSITION, GANTRY_MOTOR_ID, GANTRY_NODE_POSITIONS,
    GANTRY_SAMPLE_INTERVAL, GRIPPER_POSITIONS, HATCHES_ANALOG_INPUTS, HATCHES_CLOSE_OUTPUT_IDS,
    HATCHES_CLOSE_SET_POINTS, HATCHES_OPEN_OUTPUT_IDS, HATCHES_OPEN_SET_POINTS, HATCHES_OPEN_TIME,
    HATCHES_SLOT_ID, HATCH_CLOSE_TIMES, HEATER_OUTPUT_ID, HEATER_SLOT_ID, SEALER_ACTUATOR_ID,
    SEALER_ANALOG_INPUT, SEALER_EXTEND_ID, SEALER_EXTEND_SET_POINT, SEALER_HEATER,
    SEALER_MOVE_DOOR_TIME, SEALER_RETRACT_ID, SEALER_RETRACT_SET_POINT, SEALER_SLOT_ID,
    SEALER_TIMEOUT, TRAP_DOOR_CLOSE_OUTPUT_ID, TRAP_DOOR_OPEN_OUTPUT_ID, TRAP_DOOR_SLOT_ID,
};
use crate::manual_control::enable_and_clear_all;
use crate::recipe_handling::Ingredient;
use control_components::subsystems::bag_handling::{BagDispenser, BagSensor};
use control_components::subsystems::dispenser::{
    DispenseParameters, Dispenser, Parameters, Setpoint, WeightedDispense,
};
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{Output, RelayHBridge};
use control_components::subsystems::sealer::Sealer;
use futures::future::join_all;
use log::{error, info};
use tokio::sync::mpsc::Sender;
use tokio::sync::Mutex;
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
    bag_loaded: BagLoadedState,
    nodes: [NodeState; 4],
    bag_filled: Option<BagFilledState>,
    failures: Vec<RyoFailure>,
    run_state: RyoRunState,
    is_single_ingredient: bool,
    recipe: [Option<DispenseParameters>; 4],
}
impl Default for RyoState {
    fn default() -> Self {
        Self {
            bag_loaded: BagLoadedState::Bagless,
            nodes: array::from_fn(|_| NodeState::Ready),
            bag_filled: None,
            failures: Vec::new(),
            run_state: RyoRunState::Ready,
            is_single_ingredient: false,
            recipe: [
                Some(DEFAULT_DISPENSE_PARAMETERS),
                None,
                None,
                None,
            ]
        }
    }
}
impl RyoState {
    pub fn new() -> Self {
        Self {
            bag_loaded: BagLoadedState::Bagless,
            nodes: array::from_fn(|_| NodeState::Ready),
            bag_filled: None,
            failures: Vec::new(),
            run_state: RyoRunState::Ready,
            is_single_ingredient: false,
            recipe: [
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
                Some(DEFAULT_DISPENSE_PARAMETERS),
            ],
        }
    }

    pub fn set_run_state(&mut self, state: RyoRunState) {
        self.run_state = state;
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

    pub fn set_dispenser_recipe(&mut self, id: usize, recipe: DispenseParameters) {
        self.recipe[id] = Some(recipe);
    }

    pub fn set_recipe(&mut self, job_order: &JobOrder) {
        let ingredients = job_order.get_ingredients();
        self.recipe = ingredients.map(|ingredient| {
            match ingredient {
                Some(ing) => Some(ing.get_parameters()),
                None => None,
            }
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

    pub fn get_bag_filled_state(&self) -> Option<BagFilledState> {
        self.bag_filled.clone()
    }

    pub fn get_bag_loaded_state(&self) -> BagLoadedState {
        self.bag_loaded.clone()
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

// pub type CycleOrder = [Option<Dispenser>; 4];
// trait NewOrder {
//     fn new(ingredients: Vec<Ingredient>, io: RyoIo);
// }
// impl NewOrder for CycleOrder {
//     fn new(ingredients: Vec<Ingredient>, io: RyoIo) -> CycleOrder {
//
//     }
// }

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

pub fn make_dispense_tasks(
    recipe: [Option<DispenseParameters>; 4],
    io: RyoIo,
) -> Vec<JoinHandle<()>> {
    let mut dispensers = Vec::with_capacity(4);
    for (id, subrecipe) in recipe.iter().enumerate() {
        let params = subrecipe.clone().unwrap().parameters;
        let setpoint = subrecipe.clone().unwrap().setpoint;
        dispensers.push(make_dispenser(
            id,
            io.cc2.clone(),
            setpoint,
            params,
            io.scale_txs[id].clone(),
        ))
    }
    dispensers
        .into_iter()
        .map(|dispenser| tokio::spawn(async move { dispenser.dispense(DISPENSER_TIMEOUT).await }))
        .collect()
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

pub fn make_default_weighed_dispense_tasks(
    serving: f64,
    ids: Vec<usize>,
    io: RyoIo,
) -> Vec<JoinHandle<()>> {
    let mut dispensers = Vec::with_capacity(4);
    for id in ids {
        // let params = Parameters::default();
        let params = Parameters {
            motor_speed: 0.5,
            sample_rate: 50.,
            cutoff_frequency: 0.5,
            check_offset: 50.,
            stop_offset: 35.,
            retract_before: None,
            retract_after: Some(5.),
        };
        let set_point = Setpoint::Weight(WeightedDispense {
            setpoint: serving,
            timeout: Duration::from_secs(60),
        });
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
    make_and_open_hatch(id, io.clone()).await;
    sleep(Duration::from_millis(100)).await;
    tokio::spawn(async move {
        make_and_close_hatch(id, io).await;
    });
    // make_and_close_hatch(id, io).await;
}

pub async fn drop_bag(io: RyoIo) {
    let gantry = make_gantry(io.cc1.clone()).await;
    let _ = gantry.absolute_move(GANTRY_BAG_DROP_POSITION).await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
    let mut bag_handler = make_bag_handler(io);
    bag_handler.open_gripper().await;
    let _ = gantry.absolute_move(GANTRY_NODE_POSITIONS[2]).await;
    bag_handler.close_gripper().await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
}

pub async fn release_bag_from_sealer(io: RyoIo) {
    let mut trap_door = make_trap_door(io.clone());
    trap_door.actuate(HBridgeState::Neg).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    trap_door.actuate(HBridgeState::Off).await;
    sleep(Duration::from_millis(100)).await;
    trap_door.actuate(HBridgeState::Pos).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    trap_door.actuate(HBridgeState::Off).await;
}

pub async fn pull_before_flight(io: RyoIo) -> RyoState {
    enable_and_clear_all(io.clone()).await;
    let gantry = make_gantry(io.cc1.clone()).await;
    loop {
        sleep(Duration::from_secs(1)).await;
        if gantry.get_status().await
            == control_components::components::clear_core_motor::Status::Ready
        {
            break;
        }
    }
    for node in 0..4 {
        let motor = io.cc2.get_motor(node);
        motor.set_velocity(0.5).await;
        motor.set_acceleration(90.).await;
        motor.set_deceleration(90.).await;
    }

    // set_motor_accelerations(io.clone(), 50.).await;
    sleep(Duration::from_millis(500)).await;

    let mut set = JoinSet::new();
    let bag_handler = BagHandler::new(io.clone());

    // make_trap_door(io.clone()).actuate(HBridgeState::Pos).await;
    make_bag_handler(io.clone()).close_gripper().await;
    make_sealer(io.clone())
        .absolute_move(SEALER_RETRACT_SET_POINT)
        .await;
    info!("Sealer retracted");

    make_trap_door(io.clone()).actuate(HBridgeState::Pos).await;
    sleep(SEALER_MOVE_DOOR_TIME).await;
    make_trap_door(io.clone()).actuate(HBridgeState::Off).await;
    info!("Trap door opened");

    for id in 0..4 {
        let io_clone = io.clone();
        info!("Closing Hatch {:?}", id);
        // make_and_close_hatch(id, io_clone).await;
        set.spawn(async move {
            info!("Closing Hatch {:?}", id);
            make_and_close_hatch(id, io_clone).await;
        });
    }

    set.spawn(async move { bag_handler.dispense_bag().await });
    set.spawn(async move {
        gantry.enable().await.expect("Motor is faulted");
        let state = gantry.get_status().await;
        if state == control_components::components::clear_core_motor::Status::Moving {
            gantry.wait_for_move(Duration::from_secs(1)).await.unwrap();
        }
        let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
        gantry.wait_for_move(Duration::from_secs(1)).await.unwrap();
    });

    drop(io);
    info!("All systems go.");
    while (set.join_next().await).is_some() {}
    let mut state = RyoState::new();
    state.set_run_state(RyoRunState::Running);
    state
}

pub async fn pull_after_flight(io: RyoIo) {
    let gantry = make_gantry(io.cc1.clone()).await;
    let _ = gantry.absolute_move(GANTRY_HOME_POSITION).await;
    BagHandler::new(io.clone()).dispense_bag().await;
    gantry.wait_for_move(GANTRY_SAMPLE_INTERVAL).await.unwrap();
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

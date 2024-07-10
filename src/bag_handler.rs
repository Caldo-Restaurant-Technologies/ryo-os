use control_components::components::clear_core_io::{DigitalInput, DigitalOutput};
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper};
use std::future::Future;
use tokio::time::Duration;

use crate::config::{
    BAG_BLOWER, BAG_DETECT_PE, BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, GRIPPER_ACTUATOR,
    GRIPPER_MOTOR_ID,
};
use control_components::controllers::clear_core::Controller;
use control_components::subsystems::linear_actuator::SimpleLinearActuator;
use tokio::sync::mpsc::Receiver;

pub struct BagHandler {
    bag_dispenser: BagDispenser,
    bag_gripper: BagGripper,
    blower: DigitalOutput,
    bag_detect: DigitalInput,
}

impl BagHandler {
    pub fn new(cc1: Controller, cc2: Controller) -> Self {
        let bag_dispenser = make_bag_dispenser(cc1.clone());
        let bag_gripper = make_bag_gripper(cc1.clone(), cc2.clone());
        let blower = cc2.get_output(BAG_BLOWER);
        let bag_detect = cc1.get_digital_input(BAG_DETECT_PE);
        Self {
            bag_dispenser,
            bag_gripper,
            blower,
            bag_detect,
        }
    }

    pub async fn load_bag(&mut self) {
        load_bag(&self.bag_dispenser, &mut self.bag_gripper, &self.blower).await;
    }
    pub async fn dispense_bag(&self) {
        self.bag_dispenser.dispense().await.unwrap();
    }
}
pub enum BagHandlingCmd {
    DispenseBag,
    LoadBag,
}

pub enum ManualBagHandlingCmd {}
pub async fn actor(cc1: Controller, cc2: Controller, mut rx: Receiver<BagHandlingCmd>) {
    let mut bag_handler = BagHandler::new(cc1, cc2);
    while let Some(cmd) = rx.recv().await {
        match cmd {
            BagHandlingCmd::LoadBag => {
                bag_handler.load_bag().await;
            }
            BagHandlingCmd::DispenseBag => {
                bag_handler.dispense_bag().await;
            }
        }
    }
}

pub async fn load_bag(dispenser: &BagDispenser, gripper: &mut BagGripper, blower: &DigitalOutput) {
    blower.set_state(true).await;
    gripper.open().await;
    tokio::time::sleep(Duration::from_millis(1000)).await;
    dispenser.pull_back().await.unwrap();
    gripper.close().await;
    blower.set_state(false).await;
    gripper.rip_bag().await.unwrap();
}

pub fn make_bag_dispenser(drive: Controller) -> BagDispenser {
    BagDispenser::new(
        drive.get_motor(BAG_ROLLER_MOTOR_ID),
        drive.get_digital_input(BAG_ROLLER_PE),
    )
}
pub fn make_bag_gripper(drive_1: Controller, drive_2: Controller) -> BagGripper {
    BagGripper::new(
        drive_1.get_motor(GRIPPER_MOTOR_ID),
        SimpleLinearActuator::new(drive_2.get_h_bridge(GRIPPER_ACTUATOR)),
        [0.4, -0.8, 0.4].to_vec(),
    )
}

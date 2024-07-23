use control_components::components::clear_core_io::{DigitalInput};
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper};
use tokio::time::Duration;

use crate::config::{
    BAG_DETECT_PE, BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, BLOWER_OUTPUT_ID,
    BLOWER_SLOT_ID, ETHERCAT_RACK_ID, GRIPPER_ACTUATOR, GRIPPER_MOTOR_ID,
};
use crate::ryo::RyoIo;
use control_components::controllers::clear_core::Controller;
use control_components::controllers::ek1100_io::IOCard;
use control_components::subsystems::linear_actuator::{SimpleLinearActuator};
use tokio::sync::mpsc::Receiver;

pub struct BagHandler {
    bag_dispenser: BagDispenser,
    bag_gripper: BagGripper,
    blower: IOCard,
    bag_detect: DigitalInput,
}

impl BagHandler {
    pub fn new(mut io: RyoIo) -> Self {
        let bag_dispenser = make_bag_dispenser(io.cc1.clone());
        let bag_gripper = make_bag_gripper(io.clone());
        // let blower = Output::EtherCat(io.etc_io.get_io(ETHERCAT_RACK_ID), BLOWER_SLOT_ID, BLOWER_OUTPUT_ID as u8);
        let blower = io.etc_io.get_io(ETHERCAT_RACK_ID);
        let bag_detect = io.cc1.get_digital_input(BAG_DETECT_PE);
        Self {
            bag_dispenser,
            bag_gripper,
            blower,
            bag_detect,
        }
    }

    pub async fn load_bag(&mut self) {
        load_bag(&self.bag_dispenser, &mut self.bag_gripper, &mut self.blower).await;
    }
    pub async fn dispense_bag(&self) {
        self.bag_dispenser.dispense().await.unwrap();
    }

    pub async fn open_gripper(&mut self) {
        self.bag_gripper.open().await;
    }

    pub async fn close_gripper(&mut self) {
        self.bag_gripper.close().await;
    }
}
pub enum BagHandlingCmd {
    DispenseBag,
    LoadBag,
}

pub enum ManualBagHandlingCmd {}
pub async fn actor(io: RyoIo, mut rx: Receiver<BagHandlingCmd>) {
    let mut bag_handler = BagHandler::new(io);
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

pub async fn load_bag(dispenser: &BagDispenser, gripper: &mut BagGripper, blower: &mut IOCard) {
    // elf.actuator_io.set_state(1, self.extend_id, true).await;
    blower
        .set_state(BLOWER_SLOT_ID, BLOWER_OUTPUT_ID as u8, true)
        .await;
    gripper.open().await;
    tokio::time::sleep(Duration::from_millis(1000)).await;
    dispenser.pull_back().await.unwrap();
    gripper.close().await;
    blower
        .set_state(BLOWER_SLOT_ID, BLOWER_OUTPUT_ID as u8, false)
        .await;
    // gripper.rip_bag().await.unwrap();
}

pub fn make_bag_dispenser(drive: Controller) -> BagDispenser {
    BagDispenser::new(
        drive.get_motor(BAG_ROLLER_MOTOR_ID),
        drive.get_digital_input(BAG_ROLLER_PE),
    )
}
pub fn make_bag_gripper(io: RyoIo) -> BagGripper {
    BagGripper::new(
        io.cc1.get_motor(GRIPPER_MOTOR_ID),
        SimpleLinearActuator::new(io.cc1.get_h_bridge(GRIPPER_ACTUATOR)),
        [0.4, -0.8, 0.4].to_vec(),
    )
}

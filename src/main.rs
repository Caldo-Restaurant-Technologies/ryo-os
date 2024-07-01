use crate::config::{
    BAG_BLOWER, BAG_ROLLER_MOTOR_ID, BAG_ROLLER_PE, CC_STEP_COUNTS, GANTRY_MOTOR_ID,
    GRIPPER_ACTUATOR, GRIPPER_MOTOR_ID, NODE_D_MOTOR_ID, STEPPER_MOTOR_COUNTS,
};
use crate::hmi::UISenders;
use crate::sealer::{Sealer, SealerCommand};
use control_components::components::clear_core_io::{DigitalInput, Output, OutputState};
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::interface::tcp;
use control_components::interface::tcp::client;
use control_components::subsystems::bag_handling::{BagDispenser, BagGripper};
use control_components::subsystems::gantry::{gantry, GantryCommand};
use control_components::subsystems::hatch::Hatch;
use control_components::subsystems::linear_actuator::{
    Message, RelayHBridge, SimpleLinearActuator,
};
use control_components::subsystems::node::{DispensingParameters, Node, NodeCommand};
use hyper::Response;
use log::info;
use serde_json::Value::Array;
use std::env;
use std::time::Duration;
use tokio::join;
use tokio::sync::mpsc::Sender;
use tokio::sync::{mpsc, oneshot};

pub mod config;
pub mod hmi;
pub mod recipe_handling;
mod sealer;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    // env::set_var("RUST_BACKTRACE", "1");

    info!("Starting up...");
    // let (hmi_tx, mut hmi_rx) = mpsc::channel(1);
    // let (drive_1_tx, drive_1_rx) = mpsc::channel(10);
    // let (drive_2_tx, drive_2_rx) = mpsc::channel(10);
    // let (gantry_tx, gantry_rx) = mpsc::channel(10);
    // let (node_tx, node_rx) = mpsc::channel::<NodeCommand>(10);
    //
    // let txs = UISenders {
    //     hmi_state: hmi_tx,
    //     drive_senders: (drive_1_tx.clone(), drive_2_tx.clone()),
    //     op_senders: (node_tx, gantry_tx),
    // };
    //
    // let node_handler = tokio::spawn(async move {
    //     let node = Node::new(ClearCoreMotor::new(
    //         NODE_D_MOTOR_ID,
    //         CC_STEP_COUNTS,
    //         drive_2_tx.clone(),
    //     ));
    //     node.actor(716620, node_rx).await.unwrap();
    // });
    //
    // let gantry_handler = tokio::spawn(async move {
    //     gantry(
    //         ClearCoreMotor::new(GANTRY_MOTOR_ID, 800, drive_1_tx.clone()),
    //         gantry_rx,
    //     )
    //     .await
    // });
    //
    // let motion_handler = tokio::spawn(async move {
    //     tokio::join!(
    //         tcp::client("192.168.1.11:8888", drive_1_rx),
    //         tcp::client("192.168.1.12:8888", drive_2_rx),
    //     )
    // });
    // let state_server = tokio::spawn(hmi::ui_server(txs));
    // let server_state_handler = tokio::spawn(async move {
    //     while let Some(state) = hmi_rx.recv().await {
    //         println!("{:?}", state);
    //     }
    // });
    // let (_, _, _, _, _) = tokio::join!(
    //     state_server,
    //     server_state_handler,
    //     motion_handler,
    //     gantry_handler,
    //     node_handler
    // );

    test_cycle().await.unwrap();
    Ok(())
}

async fn test_cycle() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // Create drive channels and spawn clients
    let (drive_1_tx, drive_1_rx) = mpsc::channel(10);
    let (drive_2_tx, drive_2_rx) = mpsc::channel(10);
    let drive_1_client = tokio::spawn(client("192.168.1.11:8888", drive_1_rx));
    let drive_2_client = tokio::spawn(client("192.168.1.12:8888", drive_2_rx));

    // Create gantry channels and spawn client
    let (gantry_tx, gantry_rx) = mpsc::channel(10);
    let gantry_drive_tx = drive_1_tx.clone();
    let gantry_handler = tokio::spawn(async move {
        gantry(
            ClearCoreMotor::new(0, 800, gantry_drive_tx.clone()),
            gantry_rx,
        )
        .await
        .expect("TODO: panic message");
        ClearCoreMotor::new(0, 800, gantry_drive_tx)
            .enable()
            .await
            .unwrap();
    });

    // Create node channels and spawn client
    let (node_tx, node_rx) = mpsc::channel::<NodeCommand>(10);
    let node_drive_tx = drive_2_tx.clone();
    let node_handler = tokio::spawn(async move {
        let node = Node::new(ClearCoreMotor::new(3, 800, node_drive_tx));
        node.actor(716620, node_rx)
            .await
            .expect("TODO: panic message");
    });

    let sealer = Sealer::new(0x02, 0x01, 0x08, 0x04);
    let (sealer_tx, sealer_rx) = mpsc::channel(10);
    let sealer_handler = tokio::spawn(async move { sealer.actor(sealer_rx).await });

    // info!("Setup: Closing Hatch D");
    // close_hatch(3, drive_1_tx.clone(), drive_2_tx.clone()).await;
    // info!("Setup: Closing Sealer Door");
    close_sealer_door(sealer_tx.clone()).await;
    tokio::time::sleep(Duration::from_secs_f64(5.)).await;
    // info!("Setup: Homing Gantry");
    // gantry_to_home(gantry_tx.clone()).await;
    // info!("Dispensing");
    // dispense(node_tx.clone()).await;
    // info!("Loading bag");
    // load_bag(drive_1_tx.clone(), drive_2_tx.clone()).await;
    // info!("Moving gantry to Node D");
    // gantry_move_to(92., gantry_tx.clone()).await;
    // tokio::time::sleep(Duration::from_secs(30)).await;
    // info!("Opening Hatches");
    // open_all_hatches(drive_1_tx.clone(), drive_2_tx.clone()).await;
    // info!("Dropping bag");
    // drop_bag(drive_1_tx.clone(), drive_2_tx.clone()).await;
    info!("Applying sealer");
    apply_sealer_heater(sealer_tx.clone()).await;
    info!("Removing sealer");
    tokio::time::sleep(Duration::from_secs_f64(5.)).await;
    turn_on_heater(drive_1_tx.clone()).await;
    tokio::time::sleep(Duration::from_secs_f64(2.)).await;
    remove_sealer_heater(sealer_tx.clone()).await;
    info!("Opening Sealer Door");
    open_sealer_door(sealer_tx.clone()).await;
    info!("Test Cycle Complete!");

    let _ = {
        drive_1_tx;
        drive_2_tx;
        gantry_tx;
        node_tx;
        sealer_tx
    };
    let _ = join!(
        drive_1_client,
        drive_2_client,
        gantry_handler,
        node_handler,
        sealer_handler
    );

    Ok(())
}

async fn close_all_hatches(drive_1_tx: Sender<Message>, drive_2_tx: Sender<Message>) {
    let hatch_0 = Hatch::new(
        RelayHBridge::new(drive_1_tx.clone(), (2, 3), 3),
        Duration::from_secs(3),
    );
    let hatch_1 = Hatch::new(
        RelayHBridge::new(drive_1_tx.clone(), (4, 5), 4),
        Duration::from_secs(3),
    );
    let hatch_2 = Hatch::new(
        RelayHBridge::new(drive_2_tx.clone(), (0, 1), 3),
        Duration::from_secs(3),
    );
    let hatch_3 = Hatch::new(
        RelayHBridge::new(drive_2_tx.clone(), (2, 3), 4),
        Duration::from_secs(3),
    );

    let (_, _, _, _) = join!(
        hatch_0.timed_close(Duration::from_secs_f64(1.5)),
        hatch_1.timed_close(Duration::from_secs_f64(1.5)),
        hatch_2.timed_close(Duration::from_secs_f64(1.5)),
        hatch_3.timed_close(Duration::from_secs_f64(1.5))
    );
}

async fn turn_on_heater(drive_1_tx: Sender<Message>){
    let output = Output::new(1, drive_1_tx);
    output.set_state(OutputState::On).await.unwrap();
    tokio::time::sleep(Duration::from_secs_f64(3.)).await;
    output.set_state(OutputState::Off).await.unwrap();
}
async fn open_all_hatches(drive_1_tx: Sender<Message>, drive_2_tx: Sender<Message>) {
    let hatch_0 = Hatch::new(
        RelayHBridge::new(drive_1_tx.clone(), (2, 3), 3),
        Duration::from_secs(3),
    );
    let hatch_1 = Hatch::new(
        RelayHBridge::new(drive_1_tx.clone(), (4, 5), 4),
        Duration::from_secs(3),
    );
    let hatch_2 = Hatch::new(
        RelayHBridge::new(drive_2_tx.clone(), (0, 1), 3),
        Duration::from_secs(3),
    );
    let hatch_3 = Hatch::new(
        RelayHBridge::new(drive_2_tx.clone(), (2, 3), 4),
        Duration::from_secs(3),
    );

    let (_, _, _, _) = join!(
        hatch_0.timed_open(Duration::from_secs_f64(1.5)),
        hatch_1.timed_open(Duration::from_secs_f64(1.5)),
        hatch_2.timed_open(Duration::from_secs_f64(1.5)),
        hatch_3.timed_open(Duration::from_secs_f64(1.5))
    );
}

async fn open_hatch(hatch_id: usize, drive_1_tx: Sender<Message>, drive_2_tx: Sender<Message>) {
    let (output_pair, feedback_id, drive_tx) = match hatch_id {
        0 => ((2, 3), 3, drive_1_tx),
        1 => ((4, 5), 4, drive_1_tx),
        2 => ((0, 1), 3, drive_2_tx),
        3 => ((2, 3), 4, drive_2_tx),
        _ => {
            println!("NOT VALID HATCH ID");
            return;
        }
    };
    Hatch::new(
        RelayHBridge::new(drive_tx, output_pair, feedback_id),
        Duration::from_secs(3),
    )
    .timed_open(Duration::from_secs_f64(1.5))
    .await
    .unwrap();
}

async fn close_hatch(hatch_id: usize, drive_1_tx: Sender<Message>, drive_2_tx: Sender<Message>) {
    let (output_pair, feedback_id, drive_tx) = match hatch_id {
        0 => ((2, 3), 3, drive_1_tx),
        1 => ((4, 5), 4, drive_1_tx),
        2 => ((0, 1), 3, drive_2_tx),
        3 => ((2, 3), 4, drive_2_tx),
        _ => {
            println!("INVALID HATCH ID");
            return;
        }
    };
    Hatch::new(
        RelayHBridge::new(drive_tx, output_pair, feedback_id),
        Duration::from_secs(3),
    )
    .timed_close(Duration::from_secs_f64(1.5))
    .await
    .unwrap();
}

async fn load_bag(drive_1_tx: Sender<Message>, drive_2_tx: Sender<Message>) {
    // Create bag gripper
    let bag_gripper = ClearCoreMotor::new(GRIPPER_MOTOR_ID, 200, drive_1_tx.clone());
    bag_gripper.enable().await.unwrap();
    let bag_gripper = BagGripper::new(
        bag_gripper,
        SimpleLinearActuator::new(drive_2_tx.clone(), GRIPPER_ACTUATOR, 0),
        [-0.4, 0.8, -0.4].to_vec(),
    );

    bag_gripper.close().await.unwrap();

    // Create bag dispenser
    let bag_dispenser = ClearCoreMotor::new(
        BAG_ROLLER_MOTOR_ID,
        STEPPER_MOTOR_COUNTS,
        drive_1_tx.clone(),
    );
    bag_dispenser.enable().await.unwrap();
    let bag_dispenser = BagDispenser::new(
        bag_dispenser,
        DigitalInput::new(BAG_ROLLER_PE, drive_1_tx.clone()),
    );
    let blower = Output::new(BAG_BLOWER, drive_2_tx.clone());
    control_components::subsystems::bag_handling::load_bag(bag_dispenser, bag_gripper, blower)
        .await;
}

async fn drop_bag(drive_1_tx: Sender<Message>, drive_2_tx: Sender<Message>) {
    // Create bag gripper
    let bag_gripper = ClearCoreMotor::new(GRIPPER_MOTOR_ID, 200, drive_1_tx.clone());
    bag_gripper.enable().await.unwrap();
    let bag_gripper = BagGripper::new(
        bag_gripper,
        SimpleLinearActuator::new(drive_2_tx.clone(), GRIPPER_ACTUATOR, 0),
        [-0.4, 0.8, -0.4].to_vec(),
    );
    bag_gripper.open().await.unwrap();
}

async fn dispense(node_tx: Sender<NodeCommand>) {
    let msg = NodeCommand::Dispense(DispensingParameters::only_timeout(
        Duration::from_secs(20),
        0.3,
        50.,
        0.5,
        15.,
        7.,
    ));
    node_tx.send(msg).await.unwrap();
}

async fn gantry_move_to(position: f64, gantry_tx: Sender<GantryCommand>) {
    let msg = GantryCommand::GoTo(position);
    gantry_tx.send(msg).await.unwrap();
}

async fn gantry_to_home(gantry_tx: Sender<GantryCommand>) {
    let msg = GantryCommand::GoTo(-0.25);
    gantry_tx.send(msg).await.unwrap()
}

async fn close_sealer_door(sealer_tx: Sender<SealerCommand>) {
    sealer_tx
        .send(SealerCommand::CloseDoor(5.3))
        .await
        .expect("Open door failed");
}

async fn open_sealer_door(sealer_tx: Sender<SealerCommand>) {
    sealer_tx
        .send(SealerCommand::OpenDoor(5.3))
        .await
        .expect("Open door failed");
}

async fn apply_sealer_heater(sealer_tx: Sender<SealerCommand>) {
    sealer_tx
        .send(SealerCommand::ApplyHeater(7f64))
        .await
        .expect("Open door failed");
}

async fn remove_sealer_heater(sealer_tx: Sender<SealerCommand>) {
    sealer_tx
        .send(SealerCommand::RemoveHeater(7f64))
        .await
        .expect("Open door failed");
}

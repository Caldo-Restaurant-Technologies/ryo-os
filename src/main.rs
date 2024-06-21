use crate::config::{CC_STEP_COUNTS, GANTRY_MOTOR_ID, NODE_D_MOTOR_ID};
use crate::hmi::UISenders;
use control_components::components::clear_core_motor::ClearCoreMotor;
use control_components::interface::tcp;
use control_components::subsystems::gantry::gantry;
use control_components::subsystems::node::{Node, NodeCommand};
use tokio::sync::mpsc;

pub mod config;
pub mod hmi;
pub mod recipe_handling;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let (hmi_tx, mut hmi_rx) = mpsc::channel(1);
    let (drive_1_tx, drive_1_rx) = mpsc::channel(10);
    let (drive_2_tx, drive_2_rx) = mpsc::channel(10);
    let (gantry_tx, gantry_rx) = mpsc::channel(10);
    let (node_tx, node_rx) = mpsc::channel::<NodeCommand>(10);

    let txs = UISenders {
        hmi_state: hmi_tx,
        drive_senders: (drive_1_tx.clone(), drive_2_tx.clone()),
        op_senders: (node_tx, gantry_tx),
    };

    let node_handler = tokio::spawn(async move {
        let node = Node::new(ClearCoreMotor::new(
            NODE_D_MOTOR_ID,
            CC_STEP_COUNTS,
            drive_2_tx.clone(),
        ));
        node.actor(716620, node_rx).await.unwrap();
    });

    let gantry_handler = tokio::spawn(async move {
        gantry(
            ClearCoreMotor::new(GANTRY_MOTOR_ID, 800, drive_1_tx.clone()),
            gantry_rx,
        )
        .await
    });

    let motion_handler = tokio::spawn(async move {
        tokio::join!(
            tcp::client("192.168.1.11:8888", drive_1_rx),
            tcp::client("192.168.1.12:8888", drive_2_rx),
        )
    });
    let state_server = tokio::spawn(hmi::ui_server(txs));
    let server_state_handler = tokio::spawn(async move {
        while let Some(state) = hmi_rx.recv().await {
            println!("{:?}", state);
        }
    });
    let (_, _, _, _, _) = tokio::join!(
        state_server,
        server_state_handler,
        motion_handler,
        gantry_handler,
        node_handler
    );
    Ok(())
}

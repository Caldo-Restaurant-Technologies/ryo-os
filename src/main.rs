use tokio::sync::{mpsc};
use control_components::interface::{tcp, hmi};


pub mod config;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let (hmi_tx, mut hmi_rx) = mpsc::channel(1);
    let (_drive_1_tx, drive_1_rx) = mpsc::channel(10);
    let (_drive_2_tx, drive_2_rx) = mpsc::channel(10);
    
    let motion_control_handler = tokio::spawn(async move{
        tokio::join!(
            tcp::client("192.168.1.11:8888", drive_1_rx),
            tcp::client("192.168.1.12:8888", drive_2_rx),
        )
    });
    let state_server = tokio::spawn(hmi::ui_server(hmi_tx));
    let server_state_handler = tokio::spawn(async move {
        while let Some(state) = hmi_rx.recv().await {
            println!("{:?}", state);
        }
    });
    let (_,_,_) = tokio::join!(state_server, server_state_handler, motion_control_handler);
    Ok(())
}






use tokio::sync::{mpsc};
use crate::controls_components::hmi;


pub mod controls_components;
pub mod tcp_client;

pub mod ryo;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    // let (tx, rx) = mpsc::channel(1);
    // let server = tokio::spawn(hmi::ui_server(tx));
    // let sys_controls = tokio::spawn(hmi::ui_request_handler(rx));
    // 
    // sys_controls.await.unwrap();
    // server.await.unwrap()
    Ok(())
}






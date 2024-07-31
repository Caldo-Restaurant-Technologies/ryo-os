use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use control_components::components::scale::ScaleCmd;
use log::info;
use serde::{Deserialize, Serialize};
use tokio::net::UdpSocket;
use tokio::sync::{mpsc, Mutex, oneshot};
use tokio::time;
use crate::app_integration::SystemMode;



pub async fn serve_state(state: Arc<Mutex<SystemMode>>, shutdown: Arc<AtomicBool>) -> Result<(), io::Error> {
    let socket = UdpSocket::bind("127.0.0.1:8080").await?;
    let mut buffer = [0; 1024];
    loop {
        if shutdown.load(Ordering::Relaxed) {
            break;
        }
        if let Ok(_) = socket.recv_from(buffer.as_mut_slice()).await {
            let decoded: SystemMode = bincode::deserialize(buffer.as_slice())
                .expect("Invalid!");
            *state.lock().await = decoded;
            info!("New state received from UI: {:?}", buffer);
        }
    }
    Ok(())
}

#[derive(Serialize, Deserialize, Debug)]
struct Weights{
    values: Vec<f64>
}

pub async fn serve_weights(
    senders: &[mpsc::Sender<ScaleCmd>] , 
    shutdown: Arc<AtomicBool>
) -> Result<(), io::Error> {
    let socket = UdpSocket::bind("127.0.0.1:9090").await?;
    let mut buffer = [0;1024];
    let mut interval = time::interval(Duration::from_millis(500));
    loop {
        if shutdown.load(Ordering::Relaxed){
           break;
        }
        if let Ok((_, addr)) = socket.recv_from(buffer.as_mut_slice()).await {
            let mut  weight_arr = Vec::with_capacity(4);
            for sender in senders {
                let (tx, rx) = oneshot::channel();
                let msg = ScaleCmd(tx);
                sender.send(msg).await.expect("Sender Failed from weight server");
                let weight = rx.await.expect("Received Failed from weight server");
                weight_arr.push(weight);
            }
            let weights = Weights{values: weight_arr};
            let serialized = bincode::serialize(&weights).unwrap();
            socket.send_to(&serialized, addr).await?;
        }
        interval.tick().await;
    }
    Ok(())
}
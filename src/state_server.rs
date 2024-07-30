use std::io;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use crate::app_integration::SystemMode;


pub struct StateServer {
    socket: UdpSocket,
    buf: Vec<u8>, 
    msg_available: Option<(usize, SocketAddr)>
}

impl StateServer {
    pub async fn run(mut self, state: Arc<Mutex<SystemMode>>, shutdown: Arc<AtomicBool>){ 
        loop {
            if shutdown.load(Ordering::Relaxed) {
                break
            } 
        }
        
        if let  Some((size, peer)) = self.msg_available {
            let amt = self.socket.send_to(&self.buf[..size], &peer).await.unwrap();
        }
        
        self.msg_available = Some(self.socket.recv_from(&mut self.buf).await.unwrap());
  
    }
}
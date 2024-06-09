use std::error::Error;
use tokio::sync::{mpsc, oneshot};
use crate::controls_components::helper::{int_to_byte, int_to_bytes};
use crate::tcp_client::Message;

const STX: u8 = 2;
const CR: u8 = 13;


pub struct Input{
    id: u8,
    pub cmd: [u8;4]
}

impl Input {
    pub fn new(id: u8) -> Self {
        let cmd = [STX, b'I', int_to_byte(id), CR];
        Input{ id, cmd }
    }
    
}

pub enum OutputState {
    Off,
    On
}

pub struct Output{
    id: u8,
    on_cmd: [u8; 9],
    off_cmd: [u8; 9]
}

impl Output {
    pub fn new(id: u8) -> Self {
        let on_cmd = [STX, b'O', int_to_byte(id), b'3' , b'2', b'7', b'0', b'0', CR];
        let off_cmd = [STX, b'O', int_to_byte(id), b'0', CR, 0, 0, 0, 0];
        Output{id, on_cmd, off_cmd}
    }
    
    pub fn command_builder(&self, state: OutputState) -> [u8;9] {
        match state {
            OutputState::Off => {self.off_cmd}
            OutputState::On => {self.on_cmd}
        }
    }
}

#[derive(Debug)]
pub enum HBridgeState {
    Pos,
    Neg,
    Off
}

pub struct HBridge {
    id: u8,
    power: i16,
    prefix: [u8;3]
}

impl HBridge {
    pub fn new(id: u8, power: i16) -> Self {
        let prefix = [STX, b'O', int_to_byte(id)];
        HBridge{id, power, prefix}
    }
    
    pub fn command_builder(&self, state: HBridgeState) -> Vec<u8> {
        let state = match state {
            HBridgeState::Pos => {int_to_bytes(self.power)}
            HBridgeState::Neg => {int_to_bytes(-self.power)}
            HBridgeState::Off => {int_to_bytes(0)}
        };
        let mut cmd: Vec<u8> = Vec::with_capacity(self.prefix.len() + state.len() + 1);
        cmd.extend_from_slice(self.prefix.as_slice());
        cmd.extend_from_slice(state.as_slice());
        cmd.push(13);
        cmd
    }
}

pub struct Motor {
    id: u8,
}


pub struct Controller{
    sender: mpsc::Sender<Message>
}

impl Controller {
    pub fn new(sender: mpsc::Sender<Message>) -> Self{
        Controller{sender}
    }
    pub async fn write(&self, buffer: &[u8]) -> Result<Vec<u8>, Box<dyn Error>> {
        let (resp_tx, resp_rx) = oneshot::channel();
        let msg = Message {
            buffer: buffer.to_vec(),
            response: resp_tx
        };
        self.sender.send(msg).await?;
        let res = resp_rx.await?;
        Ok(res)
    }
}

#[tokio::test]
async fn test_controller() {
    let (tx, mut rx) = mpsc::channel::<Message>(100);
    let tx2 = tx.clone();
    let tx3 = tx.clone();
    
    let mock_client = tokio::spawn(async move {
        while let Some(msg) = rx.recv().await {
            if msg.response.send(msg.buffer).is_err() {
                eprintln!("Unable to send Response");
            }
        }
    });
    
    let controller_task_1 = tokio::spawn(async move {  
        let controller = Controller::new(tx);
        let reply = controller.write("Test_1".as_bytes()).await.expect("Failed");
        println!("{:?}", reply);
        assert_eq!(reply.as_slice(), "Test_1".as_bytes());
    });
    
    let controller_task_2 = tokio::spawn( async move {
        let controller = Controller::new(tx2);
        let reply = controller.write("Test_2".as_bytes()).await.expect("Failed");
        println!("{:?}", reply);
        assert_eq!(reply.as_slice(), "Test_2".as_bytes());
    });

    let controller_task_3 = tokio::spawn( async move {
        let controller = Controller::new(tx3);
        let reply = controller.write("Test_3".as_bytes()).await.expect("Failed");
        println!("{:?}", reply);
        assert_eq!(reply.as_slice(), "Test_3".as_bytes());
    });
    
    mock_client.await.unwrap();
    controller_task_1.await.unwrap();
    controller_task_2.await.unwrap();
    controller_task_3.await.unwrap();
}
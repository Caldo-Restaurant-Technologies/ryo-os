use std::error::Error;
use std::result::Result;
pub use std::time::Duration;
pub use tokio::sync::mpsc;
pub use crate::tcp_client::{client, Message};
use crate::controls_components::helper::{make_prefix, int_to_bytes, bytes_to_int};
use crate::controls_components::controller::{Controller};


#[derive(Debug, PartialOrd, PartialEq)]
pub enum Status {
    Disabled,
    Enabling,
    Faulted,
    Ready,
    Moving,
    Unknown
}


pub struct AsyncMotor{
    id: u8,
    prefix: [u8;3],
    scale: isize,
    drive: Controller
}


impl AsyncMotor {
    
    pub fn new(id: u8, scale: isize,  drive: Controller) -> Self {
        let prefix = make_prefix(b'M', id);
        AsyncMotor{ id, prefix, scale, drive }
    }
    
    pub async fn enable(&self) -> Result<(), Box<dyn Error>> {
        let enable_cmd = [2, b'M', self.id + 48, b'E', b'N', 13];
        self.drive.write(enable_cmd.as_ref()).await?;
        Ok(())
    }
    
    pub async fn disable(&self) -> Result<(), Box<dyn Error>> {
        let enable_cmd = [2, b'M', self.id + 48, b'D', b'E', 13];
        self.drive.write(enable_cmd.as_ref()).await?;
        Ok(())
    }
   
    pub async fn absolute_move(&self, position: isize) -> Result<(), Box<dyn Error>> {
        let position = int_to_bytes(position*self.scale);
        let mut msg: Vec<u8> = Vec::with_capacity(position.len() + self.prefix.len()+1);
        msg.extend_from_slice(self.prefix.as_slice());
        msg.extend_from_slice(b"AM");
        msg.extend_from_slice(position.as_slice());
        msg.push(13);
        self.drive.write(msg.as_slice()).await?;
        Ok(())
    }
    
    pub async fn relative_move(&self, position: isize) -> Result<(), Box<dyn Error>> {
        let position = int_to_bytes(position*self.scale);
        let mut msg: Vec<u8> = Vec::with_capacity(position.len() + self.prefix.len()+1);
        msg.extend_from_slice(self.prefix.as_slice());
        msg.extend_from_slice(b"RM");
        msg.extend_from_slice(position.as_slice());
        msg.push(13);
        self.drive.write(msg.as_slice()).await?;
        Ok(())
    }
    
    pub async fn jog(&self, speed: isize) -> Result<(), Box<dyn Error>> {
        let speed = int_to_bytes(speed * self.scale);
        let mut msg: Vec<u8> = Vec::with_capacity(speed.len() + self.prefix.len()+1);
        msg.extend_from_slice(self.prefix.as_slice());
        msg.extend_from_slice(b"JG");
        msg.extend_from_slice(speed.as_slice());
        msg.push(13);
        self.drive.write(msg.as_slice()).await?;
        Ok(())
    }
    
    pub async fn abrupt_stop(&self) -> Result<(), Box<dyn Error>> {
        let stop_cmd = [2, b'M', self.id + 48, b'A', b'S', 13];
        self.drive.write(stop_cmd.as_ref()).await?;
        Ok(())
    }

    pub async fn stop(&self) -> Result<(), Box<dyn Error>>{
        let stop_cmd = [2, b'M', self.id + 48, b'S', b'T', 13];
        self.drive.write(stop_cmd.as_ref()).await?;
        Ok(())
    }

    pub async fn set_position(&self, position: isize) -> Result<(), Box<dyn Error>> {
        let pos = int_to_bytes(position * self.scale);
        let mut msg: Vec<u8> = Vec::with_capacity(pos.len() + self.prefix.len()+1);
        msg.extend_from_slice(self.prefix.as_slice());
        msg.extend_from_slice(b"SP");
        msg.extend_from_slice(pos.as_slice());
        msg.push(13);
        self.drive.write(msg.as_slice()).await?;
        Ok(())
    }

    pub async fn set_velocity(&self, velocity: isize) -> Result<(), Box<dyn Error>> {
        let vel = int_to_bytes(velocity * self.scale);
        let mut msg: Vec<u8> = Vec::with_capacity(vel.len() + self.prefix.len()+1);
        msg.extend_from_slice(self.prefix.as_slice());
        msg.extend_from_slice(b"SV");
        msg.extend_from_slice(vel.as_slice());
        msg.push(13);
        self.drive.write(msg.as_slice()).await?;
        Ok(())
    }

    pub async fn set_acceleration(&self, acceleration: isize) -> Result<(), Box<dyn Error>> {
        let accel = int_to_bytes(acceleration * self.scale);
        let mut msg: Vec<u8> = Vec::with_capacity(accel.len() + self.prefix.len()+1);
        msg.extend_from_slice(self.prefix.as_slice());
        msg.extend_from_slice(b"SV");
        msg.extend_from_slice(accel.as_slice());
        msg.push(13);
        self.drive.write(msg.as_slice()).await?;
        Ok(())
    }

    pub async fn set_deceleration(&self, deceleration: isize) -> Result<(), Box<dyn Error>> {
        let accel = int_to_bytes(deceleration * self.scale);
        let mut msg: Vec<u8> = Vec::with_capacity(accel.len() + self.prefix.len()+1);
        msg.extend_from_slice(self.prefix.as_slice());
        msg.extend_from_slice(b"SV");
        msg.extend_from_slice(accel.as_slice());
        msg.push(13);
        self.drive.write(msg.as_slice()).await?;
        Ok(())
    }

    pub async fn get_status(&self) -> Result<Status, Box<dyn Error>> {
        let status_cmd = [2, b'M', self.id + 48, b'G', b'S', 13];
        let res = self.drive.write(status_cmd.as_slice()).await?;
        match res[3]{
            48 => {Ok(Status::Disabled)},
            49 => {Ok(Status::Enabling)},
            50 => {Ok(Status::Faulted)},
            51 => {Ok(Status::Ready)},
            54 => {Ok(Status::Moving)},
            _ => {Ok(Status::Unknown)}
        }
    }

    pub async fn get_position(&self) -> Result<isize, Box<dyn Error>> {
        let get_pos_cmd = [2, b'M', self.id + 48, b'G', b'P', 13];
        let res = self.drive.write(get_pos_cmd.as_slice()).await?;
        Ok(bytes_to_int(res.as_slice()))
    }

    pub async fn clear_alerts(&self) -> Result<(), Box<dyn Error>> {
        let clear_cmd = [2, b'M', self.id + 48, b'C', b'A', 13];
        self.drive.write(clear_cmd.as_slice()).await?;
        Ok(())
    }
}

#[tokio::test]
pub async fn test_motor_enable_disable() {
    //NOTE: It is UNSAFE to test motion unless we are right in front of Ryo therefore we're only 
    //Testing enable/disable and status in this automated test. For motion, we should test manually
    let (m1tx, rx) = mpsc::channel::<Message>(100);
    let m2tx = m1tx.clone();
    let m3tx = m1tx.clone();
    let m4tx = m1tx.clone();

    let client = tokio::spawn(client("192.168.1.11:8888", rx));
    
    let enable = tokio::spawn(async move {
        let motor1 = AsyncMotor::new(0,800, Controller::new(m1tx));
        let motor2 = AsyncMotor::new(1,800, Controller::new(m2tx));
        let motor3 = AsyncMotor::new(2, 800, Controller::new(m3tx));
        let motor4 = AsyncMotor::new(2, 800, Controller::new(m4tx));
        motor1.enable().await.expect("No msg received...");
        motor2.enable().await.expect("No msg received...");
        motor3.enable().await.expect("No msg received...");
        motor4.enable().await.expect("No msg received...");
        
        //Give clear core and ethernet time to enable
        tokio::time::sleep(Duration::from_millis(1000)).await;
        //If a motor drive is not connected then Status will return faulted unless HLFB is disabled
        //on ClearCore
        let m1_status = motor1.get_status().await.expect("No msg received...");
        assert_eq!(m1_status, Status::Ready);
        let m2_status = motor2.get_status().await.expect("No msg received...");
        assert_eq!(m2_status, Status::Ready);
        let m3_status = motor3.get_status().await.expect("No msg received...");
        assert_eq!(m3_status, Status::Ready);
        let m4_status = motor4.get_status().await.expect("No msg received...");
        assert_eq!(m4_status, Status::Ready);

        motor1.disable().await.expect("No msg received...");
        motor2.disable().await.expect("No msg received...");
        motor3.disable().await.expect("No msg received...");
        motor4.disable().await.expect("No msg received...");

        let m1_status = motor1.get_status().await.expect("No msg received...");
        assert_eq!(m1_status, Status::Disabled);
        let m2_status = motor2.get_status().await.expect("No msg received...");
        assert_eq!(m2_status, Status::Disabled);
        let m3_status = motor3.get_status().await.expect("No msg received...");
        assert_eq!(m3_status, Status::Disabled);
        let m4_status = motor4.get_status().await.expect("No msg received...");
        assert_eq!(m4_status, Status::Disabled);

    });
    client.await.unwrap().expect("TODO: panic message");
    enable.await.unwrap();
}



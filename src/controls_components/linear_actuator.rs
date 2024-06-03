#[allow(unused_imports)]
use std::sync::{Arc};
use std::error::Error;
use std::time::Duration;
#[allow(unused_imports)]
use tokio::sync::{Mutex, oneshot, mpsc};
use crate::controls_components::controller::Controller;
use crate::controls_components::helper::{bytes_to_int, int_to_bytes};
#[allow(unused_imports)]
use crate::tcp_client;
pub use crate::tcp_client::Message;


pub struct LinearActuator {
    output: u8,
    feedback: u8,
    drive: Controller,
}

impl LinearActuator {
    pub fn new(output: u8, feedback: u8, drive: Controller) -> Self {
        LinearActuator{output, feedback, drive}
    }
    
    pub async fn get_feedback(&self) -> Result<isize, Box<dyn Error>> {
        let cmd = [2, b'I', self.feedback + 48, 13];
        let res = self.drive.write(cmd.as_slice()).await?;
        println!("{:?}", res);
        Ok(bytes_to_int(&res[..]))
    }
    
    pub async fn actuate(&self, power: isize) -> Result<(), Box<dyn Error>> {
        let prefix = [2, b'O', self.output + 48];
        let power = int_to_bytes(power);
        let mut cmd: Vec<u8> = Vec::with_capacity(prefix.len() + power.len()+1);
        cmd.extend_from_slice(prefix.as_slice());
        cmd.extend_from_slice(power.as_slice());
        cmd.push(13);
        self.drive.write(cmd.as_slice()).await?;
        Ok(())
    }
}

pub enum ActuatorCh {
    Cha,
    Chb
}
pub struct MPlexActuatorPair {
    relay_out: u8,
    h_bridge: u8,
    feedback_a: u8,
    feedback_b: u8,
    drive: Controller
}

impl MPlexActuatorPair {
    pub fn new(relay_out: u8, h_bridge: u8, feedback_a: u8, feedback_b: u8, drive: Controller) -> Self {
        MPlexActuatorPair{
            relay_out,
            h_bridge,
            feedback_a,
            feedback_b,
            drive
        }
    }
 
    
    pub async fn get_feedback(&self, channel: ActuatorCh) -> Result<isize, Box<dyn Error>> {
        let feedback = match channel{
            ActuatorCh::Cha => {
                self.feedback_a
            }
            ActuatorCh::Chb => {
                self.feedback_b 
            }
        };
        let cmd = [2, b'I', feedback + 48, 13];
        let res = self.drive.write(cmd.as_slice()).await?;
        Ok(bytes_to_int(&res[3..]))
    }
    
    pub async fn actuate(&self, channel: ActuatorCh, power: isize) -> Result<(), Box<dyn Error>> {
        //Yeah, this sucks change later
        let cmd1 = match channel {
            ActuatorCh::Cha => {
                [2, b'O', self.relay_out + 48, 48 , 13].to_vec()
                
            }
            ActuatorCh::Chb => {
                [2, b'O', self.relay_out + 48, 2 + 48,5 + 48 ,5+48, 13].to_vec()
            }
        };
        println!("Relay command sent: {:?}", cmd1);
        self.drive.write(cmd1.as_slice()).await?;
        let prefix = [2, b'O', self.h_bridge + 48];
        let power = int_to_bytes(power);
        let mut cmd: Vec<u8> = Vec::with_capacity(prefix.len() + power.len()+1);
        cmd.extend_from_slice(prefix.as_slice());
        cmd.extend_from_slice(power.as_slice());
        cmd.push(13);
        self.drive.write(cmd.as_slice()).await?;
        Ok(())
    }
}

pub struct HMadeHBridge {
    ch_a: u8,
    ch_b: u8,
    output: u8,
    drive: Controller
}

impl HMadeHBridge {
    pub fn new(ch_a: u8, ch_b: u8, output: u8, drive: Controller) -> Self {
        HMadeHBridge{
            ch_a,
            ch_b,
            output,
            drive
        }
    }

   
    pub async fn actuate(&self, channel: ActuatorCh, power: isize) -> Result<(), Box<dyn Error>> {
        match channel {
            ActuatorCh::Cha => {
                let rly_off = [2, b'O', self.ch_b + 48, 0, 13];
                let rly_on = [2, b'O', self.ch_a + 48, 2,5,5, 13];
                self.drive.write(rly_off.as_slice()).await?;
                self.drive.write(rly_on.as_slice()).await?;
            }
            ActuatorCh::Chb => {
                let rly_off = [2, b'O', self.ch_a + 48, 48, 13];
                let rly_on = [2, b'O', self.ch_b + 48, 2,5,5, 13];
                self.drive.write(rly_off.as_slice()).await?;
                self.drive.write(rly_on.as_slice()).await?;
            }
        }
        let prefix = [2, b'O', self.output + 48];
        let power = int_to_bytes(power);
        let mut cmd: Vec<u8> = Vec::with_capacity(prefix.len() + power.len()+1);
        cmd.extend_from_slice(prefix.as_slice());
        cmd.extend_from_slice(power.as_slice());
        cmd.push(13);
        self.drive.write(cmd.as_slice()).await?;
        Ok(())
    }

  
    pub async fn all_off(&self) -> Result<(), Box<dyn Error>> {
        let rly_a_off = [2, b'O', self.ch_b + 48, 48, 13];
        let rly_b_off = [2, b'O', self.ch_a + 48, 48, 13];
        self.drive.write(rly_a_off.as_slice()).await?;
        self.drive.write(rly_b_off.as_slice()).await?;
        Ok(())
    }
}

#[tokio::test]
async fn integration_test(){
    let (tx, rx) = mpsc::channel::<Message>(10);

    let la_task_read_pos = tokio::spawn( async move { 
        let actuator = LinearActuator::new(5, 3 , Controller::new(tx));
        let pos = actuator.get_feedback().await.expect("Failed to read");
        println!("Actuator position: {} .", pos);
        let _ = actuator.actuate(32000).await;
        tokio::time::sleep(Duration::from_millis(3000)).await;
        let _ = actuator.actuate(0).await;
        let pos = actuator.get_feedback().await.expect("Failed to read");
        println!("Actuator position: {} .", pos);
    });
    
    let client = tokio::spawn(tcp_client::client("192.168.1.11:8888", rx));
    //let client = tokio::spawn(tcp_client::echo_client(rx));
    
    let _ = la_task_read_pos.await;
    let _ = client.await;
    
}

#[tokio::test]
async fn test_mplex_actuator() {
    let (tx, rx) = mpsc::channel::<Message>(10);
    
    let actuator_task =  tokio::spawn(async move { 
        let actuators_1 = MPlexActuatorPair::new(
            2, 4,5, 6,Controller::new(tx)
        );
        tokio::time::sleep(Duration::from_millis(5000)).await;
        let a1_fb_a = actuators_1.get_feedback(ActuatorCh::Cha).await.unwrap();
        println!("A1 CH A pos: {a1_fb_a}");
        actuators_1.actuate(ActuatorCh::Chb, -32000).await.unwrap();
        tokio::time::sleep(Duration::from_millis(3000)).await;
        actuators_1.actuate(ActuatorCh::Chb, 0).await.unwrap();
        let a1_fb_a_post = actuators_1.get_feedback(ActuatorCh::Cha).await.unwrap();
        println!("A1 CH A pos: {a1_fb_a}");
        //assert_ne!(a1_fb_a_post, a1_fb_a);
        

        // let a1_fb_b = actuators_1.get_feedback(ActuatorCh::Chb).await.unwrap();
        // println!("A1 CH B pos: {a1_fb_b}");
        // actuators_1.actuate(ActuatorCh::Chb, 32000).await.unwrap();
        // tokio::time::sleep(Duration::from_millis(3000)).await;
        // actuators_1.actuate(ActuatorCh::Chb, 0).await.unwrap();
        // let a1_fb_b_post = actuators_1.get_feedback(ActuatorCh::Chb).await.unwrap();
        // println!("A1 CH B pos: {a1_fb_b}");
        // //assert_ne!(a1_fb_b_post, a1_fb_b);

        
    });
    let client = tokio::spawn(tcp_client::client("192.168.1.12:8888", rx));
    let _ = actuator_task.await.unwrap();
    let _ = client.await.unwrap();
}
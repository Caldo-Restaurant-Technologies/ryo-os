#[allow(unused_imports)]
use std::sync::{Arc};
use std::error::Error;
use std::time::Duration;
#[allow(unused_imports)]
use tokio::sync::{Mutex, oneshot, mpsc};
use crate::controls_components::controller::{
    Controller, 
    Input, 
    Output, 
    OutputState, 
    HBridge, 
    HBridgeState
};
use crate::controls_components::helper::{bytes_to_int};
#[allow(unused_imports)]
use crate::tcp_client;
pub use crate::tcp_client::Message;

#[allow(unused)]
const ACTUONIX_LA_MAX_STROKE: isize = 34000;

#[allow(unused)]
const ACTUONIX_LA_MIN_STROKE: isize = 400;

#[allow(unused)]
const CLEAR_CORE_H_BRIDGE_MAX: i16 = 32760;


pub struct LinearActuator {
    output: HBridge,
    feedback: Input,
    drive: Controller,
}

impl LinearActuator {
    pub fn new(output: HBridge, feedback: Input, drive: Controller) -> Self {
        LinearActuator{ output, feedback, drive}
    }

    async fn actuate(&self, state: HBridgeState) -> Result<(), Box<dyn Error>> {
        self.drive.write(self.output.command_builder(state).as_slice()).await?;
        Ok(())
    }
    
    pub async fn get_feedback(&self) -> Result<isize, Box<dyn Error>> {
        let res = self.drive.write(self.feedback.cmd.as_slice()).await?;
        Ok(bytes_to_int(&res[2..]))
    }
    
    pub async fn open(&self, set_point: isize, timeout: Duration) -> Result<(), Box<dyn Error>> {
        self.actuate(HBridgeState::Pos).await?;
        let start_time = tokio::time::Instant::now();
        while self.get_feedback().await? >= set_point {
            let curr_time = tokio::time::Instant::now();
            if (curr_time - start_time) >= timeout {
                //TODO: Add some proper error handling
                println!("TIMED OUT!");
                break
            }
        }
        self.actuate(HBridgeState::Off).await?;
        Ok(())
    }

    pub async fn close(&self, set_point: isize, timeout: Duration) -> Result<(), Box<dyn Error>> {
        self.actuate(HBridgeState::Neg).await?;
        let start_time = tokio::time::Instant::now();
        while self.get_feedback().await? <= set_point {
            let curr_time = tokio::time::Instant::now();
            if (curr_time - start_time) >= timeout {
                //TODO: Add some proper error handling
                println!("TIMED OUT!");
                break
            }
        }
        self.actuate(HBridgeState::Off).await?;
        //TODO: Add Error Log or figure out what custom error to return
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub enum ActuatorCh {
    Cha,
    Chb
}
pub struct MPlexActuatorPair {
    output_pair: (Output, HBridge),
    feedback_pair: (Input, Input),
    drive: Controller
}

impl MPlexActuatorPair {
    pub fn new(output_pair: (Output,HBridge), feedback_pair:(Input,Input), drive: Controller) -> Self {
        MPlexActuatorPair{ output_pair, feedback_pair, drive }
    }
    pub async fn get_feedback(&self, channel: ActuatorCh) -> Result<isize, Box<dyn Error>> {
        let feedback = match channel{
            ActuatorCh::Cha => {
                &self.feedback_pair.0
            }
            ActuatorCh::Chb => {
                &self.feedback_pair.1
            }
        };
        let res = self.drive.write(feedback.cmd.as_slice()).await?;
        Ok(bytes_to_int(&res[2..]))
    }
    
    async fn actuate(&self, channel: ActuatorCh, power: HBridgeState) -> Result<(), Box<dyn Error>> {
        let relay_state = match channel {
            ActuatorCh::Cha => OutputState::On,
            ActuatorCh::Chb => OutputState::Off
        };
        let relay_cmd = self.output_pair.0.command_builder(relay_state);
        self.drive.write(relay_cmd.as_slice()).await?;
        let h_bridge_cmd = self.output_pair.1.command_builder(power);
        self.drive.write(h_bridge_cmd.as_slice()).await?;
        Ok(())
    }
    
    async fn all_off(&self) -> Result<(), Box<dyn Error>> {
        let h_bridge_cmd = self.output_pair.1.command_builder(HBridgeState::Off);
        self.drive.write(h_bridge_cmd.as_slice()).await?;
        let relay_cmd = self.output_pair.0.command_builder(OutputState::Off);
        self.drive.write(relay_cmd.as_slice()).await?;
        Ok(())
    }
    pub async fn open(
        &self, 
        channel: ActuatorCh, 
        set_point: isize, 
        timeout: Duration
    ) -> Result<(), Box<dyn Error>> {
        self.actuate(channel, HBridgeState::Pos).await?;
        let start_time = tokio::time::Instant::now();
        while self.get_feedback(channel).await? >= set_point {
            let curr_time = tokio::time::Instant::now();
            if (curr_time - start_time) > timeout {
                //TODO: Add some proper error handling
                println!("TIMED OUT!");
                break
            }
        }
        self.all_off().await?;
        //TODO: Add Error Log or figure out what custom error to return
        Ok(())
    }
    
    pub async fn close(&self, channel: ActuatorCh, set_point: isize, timeout: Duration) -> Result<(), Box<dyn Error>> {
        self.actuate(channel, HBridgeState::Neg).await?;
        let start_time = tokio::time::Instant::now();
        while self.get_feedback(channel).await? <= set_point {
            let curr_time = tokio::time::Instant::now();
            if (curr_time - start_time) > timeout {
                //TODO: Add some proper error handling
                println!("TIMED OUT!");
                break
            }
        }
        self.all_off().await?;
        //TODO: Add Error Log or figure out what custom error to return
        Ok(())
    }
}

pub enum RelayHBridgeOut{
    Output(Output),
    HBridge(HBridge)
}

pub struct RelayHBridge {
    fb_pair: (Input,Input),
    output_pair: (Output,Output),
    drive: Controller
}

impl RelayHBridge {
    pub fn new(fb_pair: (Input, Input), output_pair: (Output, Output), drive: Controller) -> Self {
        RelayHBridge {
            fb_pair,
            output_pair,
            drive
        }
    }

    pub async fn get_feedback(&self, channel: ActuatorCh) -> Result<isize, Box<dyn Error>> {
        let feedback = match channel{
            ActuatorCh::Cha => {
                &self.fb_pair.0
            }
            ActuatorCh::Chb => {
                &self.fb_pair.1
            }
        };
        let res = self.drive.write(feedback.cmd.as_slice()).await?;
        Ok(bytes_to_int(&res[2..]))
    }
    
    pub async fn actuate(&self, power: HBridgeState) -> Result<(), Box<dyn Error>> {
        match power {
            HBridgeState::Pos => {
                let msg_a = self.output_pair.0.command_builder(OutputState::On);
                self.drive.write(msg_a.as_slice()).await?;
            }
            HBridgeState::Neg => {
                let msg_b = self.output_pair.1.command_builder(OutputState::On);
                self.drive.write(msg_b.as_slice()).await?;
            }
            HBridgeState::Off => {
                let msg_a = self.output_pair.0.command_builder(OutputState::Off);
                let msg_b = self.output_pair.1.command_builder(OutputState::Off);
                self.drive.write(msg_a.as_slice()).await?;
                self.drive.write(msg_b.as_slice()).await?;
            }
        }
        Ok(())
    }
}

#[tokio::test]
async fn linear_actuator_feedback_test() {
    let (tx, rx) = mpsc::channel::<Message>(10);

    let la_task_read_pos = tokio::spawn( async move {
        let actuator = LinearActuator::new(
            HBridge::new(4,CLEAR_CORE_H_BRIDGE_MAX),
            Input::new(3),
            Controller::new(tx));
        let pos = actuator.get_feedback().await.expect("Failed to read");
        println!("Actuator position: {} .", pos);
    });

    let client = tokio::spawn(tcp_client::client("192.168.1.11:8888", rx));
    let _ = la_task_read_pos.await;
    let _ = client.await;
}

#[tokio::test]
async fn la_negative_dir_test() {
    let (tx, rx) = mpsc::channel::<Message>(10);
    let la_task = tokio::spawn( async move {
        let actuator = LinearActuator::new(
            HBridge::new(5,CLEAR_CORE_H_BRIDGE_MAX),
            Input::new(4),
            Controller::new(tx)
        );
        
        let _ = actuator.actuate(HBridgeState::Pos).await;
        tokio::time::sleep(Duration::from_secs(2)).await;
        let _ = actuator.actuate(HBridgeState::Off).await;
    });
    let client = tokio::spawn(tcp_client::client("192.168.1.11:8888", rx));
    let _ = la_task.await;
    let _ = client.await;
}

#[tokio::test]
async fn linear_actuator_actuation_test() {
    let (tx, rx) = mpsc::channel::<Message>(10);
    let la_task = tokio::spawn( async move {
        let actuator = LinearActuator::new(
            HBridge::new(4,CLEAR_CORE_H_BRIDGE_MAX),
            Input::new(3),
            Controller::new(tx));
        // let pos = actuator.get_feedback().await.expect("Failed to read");
        // println!("Actuator position: {} .", pos);
        // if pos < ACTUONIX_LA_MIN_STROKE {
        //     let _ = actuator.actuate(HBridgeState::Pos).await;
        //     tokio::time::sleep(Duration::from_secs(5)).await;
        //     let pos = actuator.get_feedback().await.expect("Failed to read");
        //     println!("Actuator position: {} .", pos);
        //     assert_eq!(pos > ACTUONIX_LA_MIN_STROKE, true);
        // } else {
        //     let _ = actuator.actuate(HBridgeState::Pos).await;
        //     tokio::time::sleep(Duration::from_secs(5)).await;
        //     let pos = actuator.get_feedback().await.expect("Failed to read");
        //     println!("Actuator position: {} .", pos);
        //     assert_eq!(pos < ACTUONIX_LA_MIN_STROKE, true);
        // }
        let _ = actuator.actuate(HBridgeState::Off).await;
    });
    let client = tokio::spawn(tcp_client::client("192.168.1.11:8888", rx));
    let _ = la_task.await;
    let _ = client.await;
}

#[tokio::test]
async fn integration_test(){
    let (tx, rx) = mpsc::channel::<Message>(10);
    let la_task_read_pos = tokio::spawn( async move { 
        let actuator = LinearActuator::new(
            HBridge::new(4,CLEAR_CORE_H_BRIDGE_MAX),
            Input::new(3), 
            Controller::new(tx)
        );
        tokio::time::sleep(Duration::from_secs(3)).await;
        //let _ = actuator.open(ACTUONIX_LA_MIN_STROKE, Duration::from_secs(3)).await;
        let pos = actuator.get_feedback().await.expect("Failed to read");
        println!("Actuator position: {} .", pos);
        let _ = actuator.close(ACTUONIX_LA_MAX_STROKE, Duration::from_secs(3)).await;
        let _ = actuator.actuate(HBridgeState::Off).await;
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
        

        let actuators_a_b = MPlexActuatorPair::new(
            (Output::new(2), HBridge::new(4, CLEAR_CORE_H_BRIDGE_MAX)),
            (Input::new(3), Input::new(4)),
            Controller::new(tx.clone())
        );
        

        let actuators_c_d = MPlexActuatorPair::new(
            (Output::new(3), HBridge::new(5, CLEAR_CORE_H_BRIDGE_MAX)),
            (Input::new(5), Input::new(6)),
            Controller::new(tx.clone())
        );
        let fb_a  = actuators_a_b.get_feedback(ActuatorCh::Cha).await.unwrap();
        println!("Actuator A, currently at {fb_a}.");
        actuators_a_b.close(ActuatorCh::Cha, 34000, Duration::from_secs(2)).await
            .expect("ConnectionError");
        
        //actuators_c_d.close(ActuatorCh::Cha, 54000, Duration::from_secs(3)).await.unwrap();
        tokio::time::sleep(Duration::from_secs(2)).await;
        // actuators_a_b.close(ActuatorCh::Chb, 44000, Duration::from_secs(3)).await.unwrap();
        // tokio::time::sleep(Duration::from_secs(6)).await;
        // actuators_c_d.close(ActuatorCh::Chb, 64000, Duration::from_secs(3)).await.unwrap();
        // tokio::time::sleep(Duration::from_secs(6)).await;
        // // println!("Opening!!!!");
        actuators_a_b.open(ActuatorCh::Cha, 400, Duration::from_secs(2)).await.unwrap();
        // tokio::time::sleep(Duration::from_secs(6)).await;
        //actuators_c_d.open(ActuatorCh::Cha, 600, Duration::from_secs(3)).await.unwrap();
        // tokio::time::sleep(Duration::from_secs(6)).await;
        // actuators_a_b.open(ActuatorCh::Chb, 500, Duration::from_secs(3)).await.unwrap();
        // tokio::time::sleep(Duration::from_secs(6)).await;
        // actuators_c_d.open(ActuatorCh::Chb, 700, Duration::from_secs(3)).await.unwrap();
    });
    let client = tokio::spawn(tcp_client::client("192.168.1.11:8888", rx));
    let _ = actuator_task.await.unwrap();
    let _ = client.await.unwrap();
}

#[tokio::test]
async fn test_mplex_actuator_open_close_sp() {
    let (tx, rx) = mpsc::channel::<Message>(10);
    
    let actuator_task =  tokio::spawn(async move { 
        
        let actuators_a_b = MPlexActuatorPair::new(
            (Output::new(2), HBridge::new(4, CLEAR_CORE_H_BRIDGE_MAX)),
            (Input::new(3), Input::new(4)),
             Controller::new(tx.clone())
        );
        
        
        let actuator_a_fb = actuators_a_b.get_feedback(ActuatorCh::Cha).await.unwrap();
        println!("Actuator A is at {actuator_a_fb} position.");
        let actuator_b_fb = actuators_a_b.get_feedback(ActuatorCh::Chb).await.unwrap();
        println!("Actuator B is at {actuator_b_fb} position.");
        
        let actuator_a_open = actuator_a_fb < 3500;
        let actuator_b_open = actuator_b_fb < 3500;
        
        if actuator_a_open {
            println!("Actuator A Open");
            actuators_a_b.close(
                ActuatorCh::Cha, 
                ACTUONIX_LA_MAX_STROKE, 
                Duration::from_secs(5)
            ).await.unwrap();
        } else {
            println!("Actuator A Closed");
            actuators_a_b.open(
                ActuatorCh::Cha,
                ACTUONIX_LA_MIN_STROKE,
                Duration::from_secs(5)
            ).await.unwrap();
        }

        if actuator_b_open {
            println!("Actuator B Open");
            actuators_a_b.close(
                ActuatorCh::Chb,
                ACTUONIX_LA_MAX_STROKE,
                Duration::from_secs(5)
            ).await.unwrap();
        } else {
            println!("Actuator B closed");
            actuators_a_b.open(
                ActuatorCh::Chb,
                ACTUONIX_LA_MIN_STROKE,
                Duration::from_secs(5)
            ).await.unwrap();
        }

        let actuators_c_d = MPlexActuatorPair::new(
            (Output::new(3), HBridge::new(5, CLEAR_CORE_H_BRIDGE_MAX)),
            (Input::new(5), Input::new(6)),
            Controller::new(tx.clone())
        );

        let actuator_c_fb = actuators_c_d.get_feedback(ActuatorCh::Cha).await.unwrap();
        println!("Actuator C is at {actuator_c_fb} position.");
        let actuator_d_fb = actuators_c_d.get_feedback(ActuatorCh::Chb).await.unwrap();
        println!("Actuator D is at {actuator_d_fb} position.");

        let actuator_c_open = actuator_c_fb < 3500;
        let actuator_d_open = actuator_d_fb < 3500;

        if actuator_c_open {
            println!("Actuator C Open");
            actuators_c_d.close(
                ActuatorCh::Cha,
                ACTUONIX_LA_MAX_STROKE,
                Duration::from_secs(5)
            ).await.unwrap();
        } else {
            println!("Actuator C Closed");
            actuators_c_d.open(
                ActuatorCh::Cha,
                ACTUONIX_LA_MIN_STROKE,
                Duration::from_secs(5)
            ).await.unwrap();
        }

        if actuator_d_open {
            println!("Actuator D Open");
            actuators_c_d.close(
                ActuatorCh::Chb,
                ACTUONIX_LA_MAX_STROKE,
                Duration::from_secs(5)
            ).await.unwrap();
        } else {
            println!("Actuator D Closed");
            actuators_c_d.open(
                ActuatorCh::Chb,
                ACTUONIX_LA_MIN_STROKE,
                Duration::from_secs(5)
            ).await.unwrap();
        }

        
    });
    let client = tokio::spawn(tcp_client::client("192.168.1.12:8888", rx));
    let _ = actuator_task.await.unwrap();
    let _ = client.await.unwrap();
}
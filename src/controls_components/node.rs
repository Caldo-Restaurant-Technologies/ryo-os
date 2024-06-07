use crate::controls_components::controller::Controller;
use crate::controls_components::motor::AsyncMotor;
use crate::controls_components::scale::{Scale, ScaleError};
use std::{error::Error};
use std::sync::Arc;
// use std::time::Duration;
use tokio::sync::{mpsc, Mutex};
use tokio::time::{sleep, Duration};
use tokio::{task, time};
use tokio::runtime::Runtime;
use crate::tcp_client::{client, Message};

pub struct Node {
    conveyor: Option<AsyncMotor>,
    scale: Option<Scale>,
}

pub struct Data {
    pub time: Vec<tokio::time::Duration>,
    pub weight: Vec<f64>,
}

impl Data {
    pub fn log(&mut self,
               time: tokio::time::Duration,
               weight: f64
    ) {
        self.time.push(time);
        self.weight.push(weight);
    }
}

impl Node {
    pub fn new(motor_id: u8,
               motor_scaling: isize,
               motor_drive: Option<Controller>,
               phidget_id: i32
    ) -> Result<Self, Box<dyn Error>> {
        // let conveyor = AsyncMotor::new(motor_id, motor_scaling, motor_drive);
        let scale = Scale::new(phidget_id)?;
        Ok(Self {
            // conveyor,
            conveyor: None,
            // scale,
            scale: None
        })
    }
    
    pub fn connect(&mut self) -> Result<(), Box<dyn Error>> {
        // self.scale.connect()
        Ok(())
    }
    
    pub async fn test_dispense(&self,
                               // weight: Arc<Mutex<f64>>,
                               serving: f64
    ) -> Result<Data, Box<dyn Error>> {

        let rt = Runtime::new().unwrap();

        let live_weight = Arc::new(Mutex::new(0.0));
        let weight_clone = Arc::clone(&live_weight);

        rt.spawn_blocking(move || {
            let mut scale = Scale::new(716709).unwrap();
            scale.connect().expect("Failed to attach load cells");
            loop {
                let new_weight = scale.live_weigh().expect("Weighing failed");
                let mut weight = weight_clone.blocking_lock();
                *weight = new_weight;
                std::thread::sleep(Duration::from_millis(10));
            }
        });
        let test = 0.;
        
        let motor_task = tokio::spawn(async move {
            // connect motor
            let (m1tx, rx) = mpsc::channel::<Message>(100);
            let client = tokio::spawn(client("192.168.1.12:8888", rx));
            let motor = AsyncMotor::new(0, 800, Controller::new(m1tx));
            motor.enable().await.expect("No msg received...");
            tokio::time::sleep(Duration::from_millis(1000)).await;
            let m_status = motor.get_status().await.expect("No msg received...");

            // set weight variables
            let initial_weight = {
                let live_weight = live_weight.lock().await;
                *live_weight
            };
            let target_weight = initial_weight - serving;
            let data_interval = Duration::from_millis(20);
            let initial_time = tokio::time::Instant::now();
            let end_time = initial_time + Duration::from_secs(10);

            // start motor
            motor.set_velocity(800).await.expect("Set velocity failed");
            motor.relative_move(6400000).await.expect("Relative move failed");


            let final_weight = loop {
                // pull current weight from scale task
                let current_weight = {
                    let live_weight = live_weight.lock().await;
                    *live_weight
                };
                if current_weight < target_weight {
                    break current_weight
                }
                let current_time = tokio::time::Instant::now();
                if current_time > end_time {
                    break current_weight
                }
                sleep(data_interval).await;
            };
            motor.abrupt_stop().await.expect("Stop failed");
            println!("Final Weight: {:?}", final_weight);
        });

        // let data = rt.block_on(motor_task);
        motor_task.await.expect("Failed...");
        
        Ok(Data{ time: Vec::new(), weight: Vec::new()})
        
    }
    
    
}

enum NodeError {
    ScaleError(ScaleError),
    MotorError
}

#[tokio::test]
async fn test_node_weighing() -> Result<(), Box<dyn Error>> {
    // // let (tx, rx) = mpsc::channel(1);
    // let mut node = Node::new(0, 800, None, 716620)?;
    // node.connect()?;
    // let weight = node.scale.weight_by_median(200, 50);
    // println!("Node {} Weight: {:?}", 0, weight);
    Ok(())
}

#[test]
fn test_dispense() -> Result<(), Box<dyn Error>> {
    // let (tx, rx) = mpsc::channel::<Message>(1);
    // let node = Arc::new(Mutex::new(Node::new(0, 800, None, 716620)?));
    // node.lock().await.test_dispense(50.).await.expect("TODO: panic message");
     let serving = 100.0;

    let rt = Runtime::new().unwrap();

    let live_weight = Arc::new(Mutex::new(0.0));
    let weight_clone = Arc::clone(&live_weight);

    let (m1tx, rx) = mpsc::channel::<Message>(100);
   // let client = client("192.168.1.12:8888", rx);
    
    let test = rt.spawn_blocking(move || {
        let mut scale = Scale::new(716709).unwrap();
       // std::thread::sleep(Duration::from_secs(3));
        scale.connect().expect("Failed to attach load cells");
       // std::thread::sleep(Duration::from_secs(3));
       // loop {
            let new_weight = scale.live_weigh().expect("Weighing failed");
            let mut weight = weight_clone.blocking_lock();
            *weight = new_weight;
            println!("{weight}")
           // std::thread::sleep(Duration::from_millis(100));
       // }//
    });


    // let motor_task = rt.spawn_blocking(async move {
    //     // connect motor
    //     let motor = AsyncMotor::new(0, 800, Controller::new(m1tx));
    //     motor.enable().await.expect("No msg received...");
    //     tokio::time::sleep(Duration::from_millis(1000)).await;
    //    // let m_status = motor.get_status().await.expect("No msg received...");
    // 
    //     // set weight variables
    //     let initial_weight = {
    //         let live_weight = live_weight.lock().await;
    //         *live_weight
    //     };
    //     let target_weight = initial_weight - serving;
    //     let data_interval = Duration::from_millis(20);
    //     let initial_time = tokio::time::Instant::now();
    //     let end_time = initial_time + Duration::from_secs(10);
    // 
    //     // start motor
    //     motor.set_velocity(800).await.expect("Set velocity failed");
    //     motor.relative_move(6400000).await.expect("Relative move failed");
    // 
    // 
    //     let final_weight = loop {
    //         // pull current weight from scale task
    //         let current_weight = {
    //             let live_weight = live_weight.lock().await;
    //             *live_weight
    //         };
    //         if current_weight < target_weight {
    //             break current_weight
    //         }
    //         let current_time = tokio::time::Instant::now();
    //         if current_time > end_time {
    //             break current_weight
    //         }
    //         sleep(data_interval).await;
    //     };
    //     motor.abrupt_stop().await.expect("Stop failed");
    //     println!("Final Weight: {:?}", final_weight);
    // });

        //let data = rt.block_on(motor_task);
       // motor_task.await.expect("Failed...");
    // rt.block_on(motor_task).expect("TODO: panic message");
   // rt.block_on(client).unwrap();
        // Ok(Data{ time: Vec::new(), weight: Vec::new()})
    
    Ok(())
}
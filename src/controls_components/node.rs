use crate::controls_components::controller::Controller;
use crate::controls_components::motor::AsyncMotor;
use crate::controls_components::scale::{Scale, ScaleError};
use std::{error::Error};
use std::fs::{create_dir_all, File};
use std::path::Path;
use serde::{Serialize, Deserialize};
use serde_json;
use tokio::sync::mpsc;
use tokio::time::{Duration, Instant};
use tokio::task::JoinHandle;
use crate::tcp_client::{client, Message};       


pub struct Node {
    phidget_id: i32,
    motor_id: u8
}

#[derive(Serialize, Deserialize)]
pub struct Data {
    pub time: Vec<tokio::time::Duration>,
    pub weight: Vec<f64>,
}

impl Data {
    pub fn new(init_time: Duration, init_weight: f64) -> Self {
        Self {
            time: vec![init_time],
            weight: vec![init_weight]
        }
    }
    pub fn log(&mut self,
               time: Duration,
               weight: f64
    ) {
        self.time.push(time);
        self.weight.push(weight);
    }
    
    pub fn plot(&self) {
        // Create a sample Data struct
        let data = Data {
            time: vec![Duration::from_millis(500), Duration::from_millis(500*2), Duration::from_millis(500*3)],
            weight: vec![10.5, 20.2, 30.7],
        };

        // Create the "data" directory if it doesn't exist
        let data_dir = Path::new("data");
        if !data_dir.exists() {
            create_dir_all(data_dir).expect("Failed to create data directory");
        }

        // Open a file for writing in the "data" directory
        let mut file = File::create(data_dir.join("data.json")).expect("Failed to create file");

        // Write the data to the JSON file
        serde_json::to_writer_pretty(&mut file, &data).expect("Failed to write to file");
    }
}
impl std::process::Termination for Data {
    fn report(self) -> std::process::ExitCode {
        // Implement the report method based on the semantics of your Data type
        // and return an appropriate ExitStatus value
        unimplemented!()
    }
}


impl Node {
    pub fn new(motor_id: u8,
               phidget_id: i32
    ) -> Result<Self, Box<dyn Error>> {
        Ok(Self {
            phidget_id,
            motor_id,
        })
    }
    
    pub fn connect(mut scale: Scale) -> Scale {
        scale.connect().expect("Scale failed to connect");
        scale
    }
    
    pub async fn dispense(mut scale: Scale,
                          mut motor: AsyncMotor,
                          serving: f64,
                          motor_speed: isize,
                          sample_rate: f64,
                          cutoff_frequency: f64,
    ) -> (Scale, AsyncMotor, Data) {
        // Instantiate motor handles
        let (mtx, rx) = mpsc::channel::<Message>(100);
        let client = tokio::spawn(client("192.168.1.12:8888", rx));
        
        // Set LP filter values
        let filter_period = 1. / sample_rate;
        let filter_rc = 1. / (cutoff_frequency * 2. * std::f64::consts::PI);
        let filter_a = filter_period / (filter_period + filter_rc);
        let filter_b = filter_rc / (filter_period + filter_rc);
        
        // Initialize dispense tracking variables
        let init_time = Instant::now();
        let mut curr_time = init_time;
        let mut last_sent_motor = curr_time;
        let init_weight = scale.weight_by_median(500, 100).expect("Failed to weigh scale");
        let mut curr_weight = init_weight;
        let target_weight = init_weight - serving;
        let timeout = Duration::from_secs(90);
        let send_command_delay = Duration::from_millis(250);
        let mut data = Data::new(curr_time-init_time, curr_weight);
        
        motor = AsyncMotor::get_enable_handle(motor).await.expect("Failed to enable");
        let (returned_scale, returned_motor) = loop {
            // Check for dispense completion
            if curr_weight < target_weight {
                motor = AsyncMotor::get_stop_handle(motor).await.expect("Failed");
                break (scale, motor)
            }
            // Check for timeout
            if curr_time - init_time > timeout {
                println!("WARNING: Dispense timed out!");
                break (scale, motor)
            }
            // Update dispense tracking variables
            curr_time = Instant::now();
            curr_weight = filter_a*scale.live_weigh().expect("Failed to weigh scale") + filter_b*curr_weight;
            data.log(curr_time-init_time, curr_weight);
            
            // Send new motor command if delay has been met
            if curr_time - last_sent_motor > send_command_delay {
                motor = AsyncMotor::get_relative_move_handle(motor, motor_speed, 10000).await.expect("Failed to update motor speed");
            }
        };
        (returned_scale, returned_motor, data)
    }
    
}

enum NodeError {
    ScaleError(ScaleError),
    MotorError
}

#[tokio::test]
async fn dispense() -> Data {
    let mut scale = Scale::new(716620).expect("Failed to construct scale");
    scale.connect().expect("Failed to connect scale");
    let (tx, rx) = mpsc::channel::<Message>(100);
    let client = tokio::spawn(client("192.168.1.12:8888", rx));
    let motor = AsyncMotor::new(0, 800, Controller::new(tx));
    
    let (_, _, data) = Node::dispense(scale, motor, 75., 50, 200., 0.5).await;
    data
}

use crate::controls_components::controller::Controller;
use crate::controls_components::motor::AsyncMotor;
use crate::controls_components::scale::{Scale, ScaleError};
use std::{error::Error};
use std::time::Duration;
use tokio::sync::mpsc;

pub struct Node {
    conveyor: AsyncMotor,
    scale: Scale,
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
               motor_drive: Controller,
               phidget_id: i32
    ) -> Result<Self, Box<dyn Error>> {
        let conveyor = AsyncMotor::new(motor_id, motor_scaling, motor_drive);
        let scale = Scale::new(phidget_id)?;
        Ok(Self {
            conveyor,
            scale,
        })
    }
    
    pub fn connect(&mut self) -> Result<(), Box<dyn Error>> {
        self.scale.connect()
    }

    pub async fn dispense(&self, 
                          serving: f64, 
                          sample_rate: usize,
                          cutoff_frequency: f64,
                          motor_rpm: isize, 
                          _weight_offset: f64, 
                          let_pass: usize
    ) -> Result<Data, Box<dyn Error>> {
        // Set LPF values
        let lpf_period = 1. / (sample_rate as f64);
        let lpf_rc = 1. / (cutoff_frequency * 2. * std::f64::consts::PI);
        let lpf_a = lpf_period / (lpf_period + lpf_rc);
        let lpf_b = lpf_rc / (lpf_period + lpf_rc);
    
        // Reverse motor to prime scale
        self.conveyor.enable().await?;
        self.conveyor.set_velocity(motor_rpm).await?;
        // figure out how long to prime for
        self.conveyor.relative_move(1000).await?;
    
        // Initialize dispensing variables
        let mut passed = 0;
        let initial_weight = self.scale.weight_by_median(200, 100)?;
        let initial_time = tokio::time::Instant::now();
        let target = initial_weight - serving;
        let last_sent = initial_time.duration_since(initial_time);
        let mut filtered_weight = initial_weight;
        let mut data = Data {
            time: vec![initial_time.duration_since(initial_time)],
            weight: vec![initial_weight]
        };
    
        while passed < let_pass {
            let weight = self.scale.live_weigh()?;
            filtered_weight = lpf_a*weight + lpf_b*filtered_weight;
            let curr_time = tokio::time::Instant::now() - initial_time;
            if curr_time - last_sent > Duration::from_millis(250) {
                let err = (filtered_weight-target) / serving;
                let new_rpm = err as isize * motor_rpm;
                if new_rpm > 50 && new_rpm < motor_rpm {
                    self.conveyor.set_velocity(new_rpm).await?;
                }
            }
            data.log(curr_time, filtered_weight);
            // dummy increment
            passed += 1;
        }
        let final_weight = self.scale.weight_by_median(300, 100)?;
        println!("Total dispensed: {}", final_weight);
        Ok(data)
    }

    pub async fn mock_dispense(&self,
                          _serving: f64,
                          sample_rate: usize,
                          cutoff_frequency: f64,
                          motor_rpm: isize,
                          _weight_offset: f64,
                          _let_pass: usize
    ) -> Result<Data, Box<dyn Error>> {
        // Set LPF values
        let lpf_period = 1. / (sample_rate as f64);
        let lpf_rc = 1. / (cutoff_frequency * 2. * std::f64::consts::PI);
        let lpf_a = lpf_period / (lpf_period + lpf_rc);
        let lpf_b = lpf_rc / (lpf_period + lpf_rc);

        // Reverse motor to prime scale
        self.conveyor.enable().await?;
        self.conveyor.set_velocity(motor_rpm).await?;
        // figure out how long to prime for
        self.conveyor.relative_move(1000).await?;

        // Initialize dispensing variables
        let initial_weight = self.scale.weight_by_median(200, 100)?;
        let initial_time = tokio::time::Instant::now();
        let end_time = tokio::time::Duration::from_secs(15);
        let mut filtered_weight = initial_weight;
        let mut data = Data {
            time: vec![initial_time.duration_since(initial_time)],
            weight: vec![initial_weight]
        };

        loop {
            let weight = self.scale.live_weigh()?;
            filtered_weight = lpf_a*weight + lpf_b*filtered_weight;
            let curr_time = tokio::time::Instant::now() - initial_time;
            data.log(curr_time, filtered_weight);
            if curr_time > end_time {
                break
            }
        }
        let final_weight = self.scale.weight_by_median(300, 100)?;
        println!("Total dispensed: {}", final_weight);
        Ok(data)
    }
    
}

enum NodeError {
    ScaleError(ScaleError),
    MotorError
}

#[tokio::test]
async fn test_node_weighing() -> Result<(), Box<dyn Error>> {
    let (tx, rx) = mpsc::channel(1);
    let mut node = Node::new(0, 800, Controller::new(tx), 716620)?;
    node.connect()?;
    let weight = node.scale.weight_by_median(200, 50);
    println!("Node {} Weight: {:?}", 0, weight);
    Ok(())
}
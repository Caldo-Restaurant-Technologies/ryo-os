use std::error::Error;
use std::time::Duration;
use tokio::time::Instant;
use crate::controls_components::controller::HBridgeState;
use crate::controls_components::linear_actuator::{LinearActuator};

pub struct Hatch<T: LinearActuator> {
    actuator: T,
    open_limit: u16,
    closed_limit:u16,
    timeout: Duration
}

impl<T: LinearActuator> Hatch<T> {

    pub fn new(actuator: T, open_limit: u16, closed_limit:u16, timeout: Duration) -> Self {
        Self{actuator,open_limit, closed_limit,timeout}
    }
    
    pub async fn open(&self, set_point: isize) -> Result<(), Box<dyn Error>>{
        self.actuator.actuate(HBridgeState::Pos).await?;
        let star_time = Instant::now();
        while self.actuator.get_feedback().await? >= set_point {
            let curr_time = Instant::now();
            if (curr_time - star_time) > self.timeout {
                //TODO: Add some proper error handling
                println!("Timed Out!");
                break;
            }
        }
        self.actuator.actuate(HBridgeState::Off).await?;
        Ok(())
    }
    
    pub async fn close(&self, set_point: isize) -> Result<(), Box<dyn Error>> {
        self.actuator.actuate(HBridgeState::Neg).await?;
        let star_time = Instant::now();
        while self.actuator.get_feedback().await? <= set_point {
            let curr_time = Instant::now();
            if (curr_time - star_time) > self.timeout {
                //TODO: Add some proper error handling
                println!("Timed Out!");
                break;
            }
        }
        self.actuator.actuate(HBridgeState::Off).await?;
        Ok(())
    }
}
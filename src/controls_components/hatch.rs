use std::time::Duration;
use crate::controls_components::linear_actuator::SimpleLinearActuator;

struct Hatch {
    actuator: SimpleLinearActuator,
    open_limit: u16,
    closed_limit:u16,
    timeout: Duration
}

impl Hatch {
    /// 
    /// 
    /// # Arguments 
    /// 
    /// * `actuator`: 
    /// * `open_limit`: 
    /// * `closed_limit`: 
    /// 
    /// returns: Hatch 
    /// 
    /// # Examples 
    /// 
    /// ```
    /// 
    /// ```
    pub fn new(
        actuator: SimpleLinearActuator,
        open_limit: u16, 
        closed_limit:u16, 
        timeout: Duration
    ) -> Self {
        Self{actuator,open_limit, closed_limit,timeout}
    }
    
    async fn open(&self, set_point: u16) {
        
    }
}
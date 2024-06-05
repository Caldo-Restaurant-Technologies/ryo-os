use crate::controls_components::controller::Controller;
use crate::controls_components::motor::AsyncMotor;
use crate::controls_components::scale::{Scale, ScaleError};
use std::error::Error;

struct Node {
    conveyor: AsyncMotor,
    scale: Scale
}

impl Node {
    pub fn new(motor_id: u8, motor_scaling: isize, motor_drive: Controller, phidget_id: i32) -> Self {
        let conveyor = AsyncMotor::new(motor_id, motor_scaling, motor_drive);
        let scale = Scale::new(phidget_id);
        Self {
            conveyor,
            scale,
        }
    }

    pub fn live_weigh(&self) -> Result<f64, NodeError> {
        self.scale.live_weigh().map_err(NodeError::ScaleError)
    }

    pub fn weight_by_median(&self, samples: usize, sample_rate: usize) -> Result<f64, NodeError> {
        self.scale.weight_by_median(samples, sample_rate).map_err(NodeError::ScaleError)
    }

    pub fn calibrate(&mut self, test_mass: f64) -> Result<(), NodeError> {
        self.scale.calibrate(test_mass).map_err(NodeError::ScaleError)
    }

    pub fn tare(&mut self, samples: usize, sample_rate: usize) -> Result<(), NodeError> {
        self.scale.tare(samples, sample_rate).map_err(NodeError::ScaleError)
    }

    pub async fn dispense(&self, serving: f64, sample_rate: usize, cutoff_frequency: f64,
                    motor_rpm: isize, weight_offset: f64, let_pass: usize)
        -> Result<(), Box<dyn Error>> {
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
        let passed = 0;
        let initial_weight = self.weight_by_median(200, 100)?;
        let initial_time = tokio::time::Instant::now();
        let last_sent = 0.;
        let mut filtered_weight = initial_weight;

        while passed < let_pass {
            let weight = self.scale.live_weigh()?;
            filtered_weight = lpf_a*weight + lpf_b*filtered_weight;
        }

        Ok(())
    }

}

enum NodeError {
    ScaleError(ScaleError),
    MotorError
}

// impl From<Error>
use linalg;
use phidget::{devices::VoltageRatioInput, Phidget};
use std::{thread, time, io};
use std::time::Duration;
use std::error::Error;
use linalg::{LinearSystem, MatrixError};
use linalg::MatrixError::{EmptyVector, InvalidMatrix};

const TIMEOUT: Duration = phidget::TIMEOUT_DEFAULT;


struct LoadCell {
    phidget_id: i32,
    channel_id: i32,
    vin: VoltageRatioInput,
}

impl LoadCell {
    
    pub fn new(phidget_id: i32, channel_id: i32) -> Self {
        let mut vin = VoltageRatioInput::new();
        vin.set_serial_number(phidget_id)?;
        vin.set_channel(channel_id)?;
        vin.open_wait(TIMEOUT)?;
        let min_data_interval = vin.min_data_interval()?;
        vin.set_data_interval(min_data_interval)?;
        thread::sleep(Duration::from_millis(3000));
        println!("Channel {:} set for Phidget {:}", channel_id, phidget_id);
        
        Self { phidget_id, channel_id, vin }
    }
    fn get_reading(&self) -> Result<f64, ScaleError> {
        // Gets the reading of a load cell from
        // Phidget.
        let reading = self.vin.voltage_ratio()?;
        Ok(reading)
    }
}

pub struct Scale {
    phidget_id: i32,
    cells: [LoadCell; 4],
    cell_coefficients: Vec<Vec<f64>>,
    tare_offset: f64,
}

impl Scale {
    pub fn new(phidget_id: i32) -> Self {
        let cells = [
            LoadCell::new(phidget_id, 0),
            LoadCell::new(phidget_id, 1),
            LoadCell::new(phidget_id, 2),
            LoadCell::new(phidget_id, 3)
        ];
        
        Self {
            phidget_id,
            cells,
            // filler coefficients for now
            cell_coefficients: vec![vec![0.]; 4],
            tare_offset: 0.
        }
    }
    
    fn get_readings(&self) -> Result<Vec<Vec<f64>>, ScaleError> {
        // Gets each load cell reading from Phidget
        // and returns them in a matrix.

        self.cells.iter().map(|cell| {
            cell.get_reading().map(|reading| vec![reading])
        }).collect()
    }

    pub fn live_weigh(&self) -> Result<f64,  ScaleError> {
        // Gets the instantaneous weight measurement
        // from the scale by taking the sum of each
        // load cell's reading, weighted by its 
        // coefficient.

        let readings = self.get_readings()?;
        let weight = LinearSystem::multiply(&readings, &self.cell_coefficients)
            .map_err(ScaleError::MatrixError)?;
        LinearSystem::unpack(weight)
            .map_err(ScaleError::MatrixError)
        
    }


    pub fn weight_by_median(&self, samples: usize, sample_rate: usize) -> Result<f64, ScaleError> {
        let mut weights = Vec::new();
        let delay = time::Duration::from_millis(1000/sample_rate as u64);
        let _start_time = time::Instant::now();
        for _sample in 0..samples {
            weights.push(self.live_weigh()?);
            thread::sleep(delay);
        }
        Ok(Scale::median(&mut weights))
    }

    fn median(weights: &mut Vec<f64>) -> f64 {
        weights.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let middle = weights.len() / 2;
        weights[middle]

    }

    pub fn tare(&mut self, samples: usize, sample_rate: usize) -> Result<(), ScaleError> {
        match self.weight_by_median(samples, sample_rate) {
            Ok(weight) => {
                self.tare_offset += weight;
                Ok(())
            }
            Err(error) => Err(error)
        }
    }

    pub fn calibrate(&mut self, test_mass: f64) -> Result<(), ScaleError> {
        let mut trial_readings = vec![vec![0.; self.cells.len()]; self.cells.len()];
        let test_mass_vector = vec![vec![test_mass]; self.cells.len()];
        for trial in 0..self.cells.len() {
            println!("Place/move test mass and press key");
            let mut input = String::new();
            match io::stdin().read_line(&mut input) {
                Ok(_user_input) => {
                    println!("Weighing...");
                    let readings = self.get_readings()?;
                    trial_readings[trial].clone_from(&readings[0]);
                },
                Err(error) => return Err(ScaleError::IoError(error))
            }
        }
        let mut system = LinearSystem::new(trial_readings, test_mass_vector).map_err(|_arg: &str| ScaleError::MatrixError(InvalidMatrix))?;
        self.cell_coefficients = system.solve();
        
        Ok(())
    }

}

#[derive(Debug)]
pub enum ScaleError {
    LoadCellError,
    MatrixError(MatrixError),
    IoError(std::io::Error),
}
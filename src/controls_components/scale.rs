use linalg;
// use phidget;
use std::{thread, time, io};
use linalg::{LinearSystem, MatrixError};
use linalg::MatrixError::{EmptyVector, InvalidMatrix};


struct LoadCell {
    phidget_id: usize,
    channel_id: usize
}

impl LoadCell {
    fn get_reading(&self) -> Result<f64, ScaleError> {
        // Gets the reading of a load cell from
        // Phidget.

        Ok(0.)
    }
}

pub struct Scale {
    cells: [LoadCell; 4],
    cell_coefficients: Vec<Vec<f64>>,
    tare_offset: f64,
}

impl Scale {
    fn get_readings(&self) -> Result<Vec<Vec<f64>>, ScaleError> {
        // Gets each load cell reading from Phidget
        // and returns them in a matrix.

        self.cells.iter().map(|cell| {
            cell.get_reading().map(|reading| vec![reading])
        }).collect()
    }

    pub fn live_weigh(&self) -> Result<f64, ScaleError> {
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
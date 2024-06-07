use linalg;
use phidget::{devices::VoltageRatioInput, Phidget};
use std::{thread, time, io};
use std::time::Duration;
use std::error::Error;
use linalg::{LinearSystem, MatrixError};
use linalg::MatrixError::{EmptyVector, InvalidMatrix};
use tokio::sync::mpsc;
use crate::controls_components::controller::Controller;
use crate::controls_components::node::Node;

const TIMEOUT: Duration = phidget::TIMEOUT_DEFAULT;
// const TIMEOUT: Duration = Duration::from_secs(3);

struct LoadCell {
    phidget_id: i32,
    channel_id: i32,
    vin: VoltageRatioInput,
}
impl LoadCell {
    
    pub fn new(phidget_id: i32, channel_id: i32) -> Result<Self, Box<dyn Error>> {
        let vin = VoltageRatioInput::new();
        Ok(Self { phidget_id, channel_id, vin })
    }

    pub fn connect(&mut self) -> Result<(), Box<dyn Error>> {
        self.vin.set_serial_number(self.phidget_id)?;
        self.vin.set_channel(self.channel_id)?;
        self.vin.open_wait(TIMEOUT).expect("Load Cell Attachment Failed");
        let min_data_interval = self.vin.min_data_interval()?;
        self.vin.set_data_interval(min_data_interval)?;
        thread::sleep(Duration::from_millis(3000));
        println!("Channel {:} set for Phidget {:}", self.channel_id, self.phidget_id);
        Ok(())
    }

    fn get_reading(&self) -> Result<f64, Box<dyn Error>> {
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
    pub fn new(phidget_id: i32) -> Result<Self, Box<dyn Error>> {
        let cells = [
            LoadCell::new(phidget_id, 0)?,
            LoadCell::new(phidget_id, 1)?,
            LoadCell::new(phidget_id, 2)?,
            LoadCell::new(phidget_id, 3)?
        ];
        
        Ok(Self {
            phidget_id,
            cells,
            // filler coefficients for now
            cell_coefficients: vec![vec![1.]; 4],
            tare_offset: 0.
        })
    }

    pub fn connect(&mut self) -> Result<(), Box<dyn Error>> {
        self.cells.iter_mut().for_each(|cell| cell.connect().expect(
            "Load Cell Attachment Failed"
        ));
        Ok(())
    }
    
    fn get_readings(&self) -> Result<Vec<Vec<f64>>, Box<dyn Error>> {
        // Gets each load cell reading from Phidget
        // and returns them in a matrix.

        self.cells.iter().map(|cell| {
            cell.get_reading().map(|reading| vec![reading])
        }).collect()
    }

    pub fn live_weigh(&self) -> Result<f64,  Box<dyn Error>> {
        // Gets the instantaneous weight measurement
        // from the scale by taking the sum of each
        // load cell's reading, weighted by its 
        // coefficient.

        let readings = self.get_readings()?;
        // println!("DEBUG:");
        // println!("Readings: {:?}", readings);
        // println!("Coefficients: {:?}", self.cell_coefficients);
        let weight = LinearSystem::dot(&readings, &self.cell_coefficients)?;
        Ok(weight)
        
    }


    pub fn weight_by_median(&self, samples: usize, sample_rate: usize) -> Result<f64, Box<dyn Error>> {
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

    pub fn tare(&mut self, samples: usize, sample_rate: usize) -> Result<(), Box<dyn Error>> {
        let weight = self.weight_by_median(samples, sample_rate)?;
        self.tare_offset += weight;
        Ok(())
    }

    pub fn calibrate(&mut self, test_mass: f64) -> Result<(), Box<dyn Error>> {
        let mut trial_readings = vec![vec![0.; self.cells.len()]; self.cells.len()];
        let test_mass_vector = vec![vec![test_mass]; self.cells.len()];
        for trial in 0..self.cells.len() {
            println!("Place/move test mass and press key");
            let mut input = String::new();
            let _user_input = io::stdin().read_line(&mut input);
            println!("Weighting...");
            let readings = self.get_readings()?;
            trial_readings[trial].clone_from(&readings[0]);
        }
        let mut system = LinearSystem::new(trial_readings, test_mass_vector)?;
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

#[test]
fn create_load_cell() -> Result<(), Box<dyn Error>> {
    let cell = LoadCell::new(716709, 0)?;
    Ok(())
}

#[test]
fn connect_load_cell() -> Result<(), Box<dyn Error>> {
    let mut cell = LoadCell::new(716709, 0)?;
    cell.connect()?;
    Ok(())
}

#[test]
fn read_load_cell() -> Result<(), Box<dyn Error>> {
    let mut cell = LoadCell::new(716709, 0)?;
    cell.connect()?;
    let reading = cell.get_reading()?;
    println!("Load Cell Reading: {:?}", reading);
    Ok(())
}

#[test]
fn create_scale() -> Result<(), Box<dyn Error>> {
    let scale = Scale::new(716709)?;
    // println!("{:?}", scale.phidget_id);
    Ok(())
}

#[test]
fn connect_scale_cells() -> Result<(), Box<dyn Error>> {
    let mut scale = Scale::new(716709)?;
    scale.connect()?;
    Ok(())
}

#[test]
fn read_scale() -> Result<(), Box<dyn Error>> {
    let mut scale = Scale::new(716709)?;
    scale.connect()?;
    let readings = scale.get_readings()?;
    println!("Scale Readings: {:?}", readings);
    Ok(())
}

#[test]
fn live_weigh_scale() -> Result<(), Box<dyn Error>> {
    let mut scale = Scale::new(716709)?;
    scale.connect()?;
    scale.cell_coefficients = vec![vec![-4832237.786999262],
                                   vec![-2679438.3255438516],
                                   vec![-4443388.581829642],
                                   vec![-4666590.62744391],
    ];
    let weight = scale.live_weigh()?;
    println!("Weight: {:?}", weight);

    Ok(())
}

#[test]
fn weigh_scale() -> Result<(), Box<dyn Error>> {
    let mut scale = Scale::new(716709)?;
    scale.connect()?;
    scale.cell_coefficients = vec![vec![-4832237.786999262],
                                   vec![-2679438.3255438516],
                                   vec![-4443388.581829642],
                                   vec![-4666590.62744391],
    ];
    let weight = scale.weight_by_median(300, 50)?;
    println!("Weight: {:?}", weight);
    
    Ok(())
}
use linalg;
use phidget::{devices::VoltageRatioInput, Phidget};
use std::{thread, time, io};
use std::time::Duration;
use std::error::Error;
use linalg::{LinearSystem, MatrixError};

const TIMEOUT: Duration = phidget::TIMEOUT_DEFAULT;

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
    pub cell_coefficients: Vec<Vec<f64>>,
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
        self.cells.iter_mut().for_each(|cell| { 
            cell.connect().expect("Load Cell Attachment Failed"); 
        });
        Ok(())
    }
    
    fn get_readings(&self) -> Result<Vec<Vec<f64>>, Box<dyn Error>> {
        // Gets each load cell reading from Phidget
        // and returns them in a matrix.

        self.cells.iter().map(|cell| {
            cell.get_reading().map(|reading| vec![reading])
        }).collect()
    }

    fn get_medians(&self, samples: usize, sample_rate: usize) -> Result<Vec<Vec<f64>>, Box<dyn Error>> {
        let mut readings: Vec<Vec<f64>> = vec![vec![]; 4];
        let mut medians = vec![0.; 4];
        let delay = time::Duration::from_millis(1000/sample_rate as u64);
        let _start_time = time::Instant::now();
        for _sample in 0..samples {
            for cell in 0..self.cells.len() {
                readings[cell].push(self.cells[cell].get_reading()?);
            }
            thread::sleep(delay);
        }
        for cell in 0..self.cells.len() {
            medians[cell] = Scale::median(&mut readings[cell]);
        }
        
        Ok(vec![medians])
    }

    pub fn live_weigh(&self) -> Result<f64,  Box<dyn Error>> {
        // Gets the instantaneous weight measurement
        // from the scale by taking the sum of each
        // load cell's reading, weighted by its 
        // coefficient.

        let readings = self.get_readings()?;
        let weight = LinearSystem::dot(&readings, &self.cell_coefficients)? - self.tare_offset;
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
        let resting_weight = self.weight_by_median(samples, sample_rate)?;
        self.tare_offset += resting_weight;
        Ok(())
    }

    pub fn calibrate(&mut self, test_mass: f64, samples: usize, sample_rate: usize) -> Result<(), Box<dyn Error>> {
        let mut trial_readings = vec![vec![0.; self.cells.len()]; self.cells.len()];
        let test_mass_vector = vec![vec![test_mass]; self.cells.len()];
        for trial in 0..self.cells.len() {
            println!("Place/move test mass and press key");
            let mut input = String::new();
            let _user_input = io::stdin().read_line(&mut input);
            println!("Weighing...");
            let readings = self.get_medians(samples, sample_rate)?;
            trial_readings[trial].clone_from(&LinearSystem::transpose(&readings)[0]);
        }
        println!("DEBUG: {:?}, {:?}", trial_readings, test_mass_vector);
        let mut system = LinearSystem::new(trial_readings, test_mass_vector)?;
        system.display();
        self.cell_coefficients = system.solve().expect("Failed to solve");
        
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

#[test]
fn calibrate_scale() -> Result<(), Box<dyn Error>> {
    let mut scale = Scale::new(716709)?;
    scale.connect()?;
    scale.calibrate(437.7, 1000, 100)?;

    Ok(())
}

#[test]
fn get_medians() -> Result<(), Box<dyn Error>> {
    let mut scale = Scale::new(716709)?;
    scale.connect()?;
    let medians = scale.get_medians(1000, 50)?;
    println!("Medians: {:?}", medians);
    Ok(())
}

// Medians: [3.1628645956516266e-5, -0.0007064277306199074, -3.0700117349624634e-5, -0.0004332391545176506] [0]
// Medians: [-3.259629011154175e-6, -0.0007126964628696442, 4.627741873264313e-6, -0.0005068713799118996] [393.2]
// Medians: [6.248243153095245e-6, -0.0007541924715042114, -3.686174750328064e-6, -0.0004650913178920746] [393.2]
// Medians: [1.3500452041625977e-5, -0.0007261056452989578, -3.0260533094406128e-5, -0.0004736315459012985] [393.2]
// Medians: [2.6219524443149567e-5, -0.0007573459297418594, -4.6545639634132385e-5, -0.00043762288987636566] [393.2]
// Medians: [-1.4428049325942993e-5, -0.00070938840508461, 1.701340079307556e-5, -0.000512399710714817] [393.2]
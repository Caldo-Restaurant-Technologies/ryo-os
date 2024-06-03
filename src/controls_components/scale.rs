use linalg;
use std::io::Error;

struct LoadCell {
    phidget_id: usize,
    channel_id: usize
}

impl LoadCell {
    fn get_reading(&self) -> Result<f64, Error> {
        // Gets the reading of a load cell from
        // Phidget.

        Ok(0.)
    }
}

pub struct Scale {
    cells: [LoadCell; 4],
    coefficients: Vec<Vec<f64>>,
}

impl Scale {
    fn get_readings(&self) -> Result<Vec<Vec<f64>>, Error> {
        // Gets each load cell reading from Phidget
        // and returns them in an array. 
        
        let mut readings = vec![vec![0.; 1]; 4];
        for cell in 0..self.cells.len() {
            // readings[cell] = self.cells.get(cell).unwrap().get_reading();
            match self.cells.get(cell).unwrap().get_reading() {
                Ok(reading) => {
                    readings[cell] = vec![reading];
                },
                Err(error) => panic!("Problem reading load cell: {:?}", error),
            }
        }
        Ok(readings)
    }

    fn live_weigh(&self) -> Result<f64, std::Error> {
        // Gets the instantaneous weight measurement
        // from the scale by taking the sum of each
        // load cell's reading, weighted by its 
        // coefficient.
        let mut readings = vec![vec![0.; 1]; 4];
        for cell in 0..self.cells.len() {
            // readings[cell] = self.cells.get(cell).unwrap().get_reading();
            match self.cells.get(cell).unwrap().get_reading() {
                Ok(reading) => {
                    readings[cell] = vec![reading];
                },
                Err(error) => panic!("Problem reading load cell: {:?}", error),
            }
        }
        Ok(linalg::LinearSystem::multiply(&readings, &self.coefficients).first().unwrap())
    }

}

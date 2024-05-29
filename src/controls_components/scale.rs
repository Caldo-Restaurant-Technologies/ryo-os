use ndarray::prelude::*;
use ndarray::linalg::Dot;
use ndarray_linalg::Solve;

struct LoadCell {
    phidget_id: usize,
    channel_id: usize
}

impl LoadCell {
    async fn get_reading(&self) -> f64 {
        // Gets the reading of a load cell from
        // Phidget.

        0.0
    }
}

pub struct Scale {
    cells: [LoadCell; 4],
    coefficients: Array2<f64>,
}

impl Scale {
    async fn get_readings(&self) -> Array2<f64> {
        // Gets each load cell reading from Phidget
        // and returns them in an array. 
        
        let arr = Array2::from_elem((4, 1), 0.0);
        arr
    }

    async fn live_weigh(&self) -> f64 {
        // Gets the instantaneous weight measurement
        // from the scale by taking the sum of each
        // load cell's reading, weighted by its 
        // coefficient.
        let readings = self.get_readings().await;
        // weight = mat.dot{}

        0.0
    }

}

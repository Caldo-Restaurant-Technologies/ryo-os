
pub enum MotionUnits {
    Revolutions(usize),
    Degrees(usize),
    Steps(usize),
    Meters(usize),
    Millimeters(usize)
}

impl MotionUnits {
    pub fn to_steps(&self, scale: usize) -> usize {
        match self {
            MotionUnits::Revolutions(pos) => {
                pos*scale
            }
            MotionUnits::Degrees(pos) => {
                pos*(scale/360)
            }
            MotionUnits::Steps(pos) => {
                *pos
            }
            MotionUnits::Meters(pos) => {
                pos*scale
            }
            MotionUnits::Millimeters(pos) => {
                *pos
            }
        }
    }
}
pub enum VelocityUnits {
    RPM,
    DegreesPerSecond,
    StepsPerSecond,
    MetersPerSecond,
    MillimetersPerSecond
}

#[allow(dead_code)]
pub struct StepsPerSecondSq(u32);

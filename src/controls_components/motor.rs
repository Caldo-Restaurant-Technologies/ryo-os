mod motor {
    use std::result::Result;
    use crate::Message;

    pub enum Error{

    }

    enum MotionUnits {
        Revolutions,
        Degrees,
        Steps,
        Meters,
        Millimeters
    }

    enum VelocityUnits {
        RPM,
        DegreesPerSecond,
        StepsPerSecond,
        MetersPerSecond,
        MillimetersPerSecond
    }

    enum Status {
        Ready,
        Moving,
        Faulted
    }
    struct StepsPerSecondSq(u32);
    enum ConnectionType{
        Socket(std::net::TcpStream),
        Channels(std::sync::mpsc::Sender<Message>)
    }

    enum AsyncConnectionType {
        SharedStream(Arc<Mutex<tokio::net::TcpStream>>),
        Channels(tokio::mpsc::Sender<Message>)
    }


    struct Motor {
        id: u8,
        name: &'static str,
        connection: ConnectionType
    }

    struct AsyncMotor<T> {
        id: u8,
        name: &'static str,
        connection: AsyncConnectionType
    }

    //Sync motor traits
    pub trait MotorControl {
        type Drive;
        fn new(id: u8, name: &'static str, connection: ConnectionType) -> Self;
        fn enable(&self) -> Result<(), Error>;
        fn disable(&self) -> Result<(), Error>;
        fn absolute_move(&self, position: MotionUnits) -> Result<(), Error>;
        fn relative_move(&self, position: MotionUnits) -> Result<(), Error>;
        fn jog(&self, speed: VelocityUnits) -> Result<(), Error>;
        fn abrupt_stop(&self) -> Result<(), Error>;
        fn stop(&self) -> Result<(), Error>;
        fn set_position(&self, position: MotionUnits::Steps) -> Result<(), Error>;
        fn set_velocity(&self, velocity: VelocityUnits) -> Result<(), Error>;
        fn set_acceleration(&self, acceleration: StepsPerSecondSq) -> Result<(), Error>;
        fn set_deceleration(&self, deceleration: StepsPerSecondSq) -> Result<(), Error>;
        fn get_status(&self) -> Result<Status, Error>;
        fn get_position(&self) -> Result<MotionUnits::Steps, Error>;
        fn clear_alerts(&self) -> Result<(), Error>;
    }

    pub trait AsyncMotorControl {
        type Drive;
        fn new(id: u8, name: &'static str, connection: ConnectionType) -> Self;
        async fn enable(&self) -> Result<(), Error>;
        async fn disable(&self) -> Result<(), Error>;
        async fn absolute_move(&self, position: MotionUnits) -> Result<(), Error>;
        async fn relative_move(&self, position: MotionUnits) -> Result<(), Error>;
        async fn jog(&self, speed: VelocityUnits) -> Result<(), Error>;
        async fn abrupt_stop(&self) -> Result<(), Error>;
        async fn stop(&self) -> Result<(), Error>;
        async fn set_position(&self, position: MotionUnits::Steps) -> Result<(), Error>;
        async fn set_velocity(&self, velocity: VelocityUnits) -> Result<(), Error>;
        async fn set_acceleration(&self, acceleration: StepsPerSecondSq) -> Result<(), Error>;
        async fn set_deceleration(&self, deceleration: StepsPerSecondSq) -> Result<(), Error>;
        async fn get_status(&self) -> Result<Status, Error>;
        async fn get_position(&self) -> Result<MotionUnits::Steps, Error>;
        async fn clear_alerts(&self) -> Result<(), Error>;
    }


    impl MotorControl for Motor {

    }

    //Async motor traits
    impl AsyncMotorControl for Motor {
        fn new(id: u8, name: &'static str, connection: ConnectionType) -> Self {
            match connection {

            }
        }
        async fn enable(&self) -> Result<(), Error> {
            todo!()
        }

    }


}



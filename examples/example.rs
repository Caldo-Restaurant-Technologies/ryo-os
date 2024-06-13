use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time;
use tokio::time::Instant;
use ryo_os::controls_components::controller::Controller;
use ryo_os::controls_components::motor::{AsyncMotor, Status};
use ryo_os::controls_components::scale::Scale;
use ryo_os::controls_components::node::{Data, Node};
use ryo_os::tcp_client::{client, Message};

const PHIDGET_SERIAL: i32 = 716709;
#[tokio::main]
async fn main() {
    // weigh().await;
    // calibrate();
    // slew().await;
    filtered_weigh_loop().await;
    

}

async fn weigh() -> Scale {
    let mut scale = Scale::new(PHIDGET_SERIAL).expect("Failed to instantiate scale");
    // [1326908.7303767086, -6930311.246202072, -712986.1175748759, 6355610.820736418, -10613.7142665367]
    scale.cell_coefficients = vec![
        vec![ 1326908.7303767086],
        vec![ -6930311.246202072],
        vec![ -712986.1175748759],
        vec![  6355610.820736418],
    ];
    let mut connected_scale = tokio::task::spawn_blocking(move || {
        scale.connect().expect("Failed to connect scale");
        scale
    })
        .await
        .expect("Failed to connect");
    // connected_scale.tare(300, 50).expect("Failed to tare");
    // let weight = connected_scale.live_weigh().expect("Failed to weigh");
    // println!("Weight: {:?}", weight);
    loop {
        let weight = connected_scale.live_weigh().expect("Failed to weigh");
        println!("Weight: {:.1}", weight-10611.638816704555-60.);
        tokio::time::sleep(Duration::from_millis(500)).await;
    }

    connected_scale
}

async fn filtered_weigh_loop() {
    let mut scale = Scale::new(PHIDGET_SERIAL).expect("Failed to instantiate scale");
    // [1326908.7303767086, -6930311.246202072, -712986.1175748759, 6355610.820736418, -10613.7142665367]
    scale.cell_coefficients = vec![
        // 21690514.0394183,-3876783.0514162118,5568955.308979326,-15454980.100074532
        vec![ 21690514.0394183],
        vec![ -3876783.0514162118],
        vec![  5568955.308979326],
        vec![-15454980.100074532],
    ];
    let mut connected_scale = tokio::task::spawn_blocking(move || {
        scale.connect().expect("Failed to connect scale");
        scale
    })
        .await
        .expect("Failed to connect");
    // connected_scale.tare(300, 50).expect("Failed to tare");
    // let weight = connected_scale.live_weigh().expect("Failed to weigh");
    // println!("Weight: {:?}", weight);
    let display_delay = 500; // ms
    let mut last_displayed = time::Instant::now();
    let sample_rate = 50.;
    let cutoff = 0.5;
    let period = 1. / sample_rate;
    let rc = 1. / (cutoff * 2. * std::f64::consts::PI);
    let a = period / (period + rc);
    let b = rc / (period + rc);
    let mut last_weight = connected_scale.live_weigh().unwrap();
    loop {
        let curr_weight = connected_scale.live_weigh().expect("Failed to weigh");
        let weight = a*curr_weight + b*last_weight;
        // println!("DEBUG: {:?}", time::Instant::now() - last_displayed);
        if time::Instant::now() - last_displayed > Duration::from_millis(display_delay) {
            last_displayed = time::Instant::now();
            println!("Weight: {:.1}", weight-10611.638816704555-60.-15.);
        }
        last_weight = curr_weight;
        tokio::time::sleep(Duration::from_millis((period*1000.) as u64)).await;
    }
}

fn calibrate() {
    let mut scale = Scale::new(PHIDGET_SERIAL).expect("Failed to instantiate scale");
    scale.connect().expect("Failed to connect scale");
    scale.calibrate(437.7, 500, 50).expect("Failed to calibrate scale");
    let coef = scale.cell_coefficients;
    println!("Coefficients: {:?}", coef);
}

async fn dispense() -> Data {
    let mut scale = Scale::new(PHIDGET_SERIAL).expect("Failed to construct scale");
    scale.connect().expect("Failed to connect scale");
    let (tx, rx) = mpsc::channel::<Message>(100);
    let client = tokio::spawn(client("192.168.1.12:8888", rx));
    let motor = AsyncMotor::new(0, 800, Controller::new(tx));

    let (_, _, data) = Node::dispense(scale, motor, 75., 50, 200., 0.5).await;
    data
}

async fn slew() {
    let (m1tx, rx) = mpsc::channel::<Message>(100);

    let client = tokio::spawn(client("192.168.1.11:8888", rx));

    let enable = tokio::spawn(async move {
        let motor1 = AsyncMotor::new(0,800, Controller::new(m1tx));
        motor1.enable().await.expect("No msg received...");
        motor1.set_velocity(6400).await.expect("No msg received...");
        motor1.relative_move(6400).await.expect("No msg received...");
        //Give clear core and ethernet time to enable
        tokio::time::sleep(Duration::from_millis(1000)).await;
        //If a motor drive is not connected then Status will return faulted unless HLFB is disabled
        //on ClearCore
        let m1_status = motor1.get_status().await.expect("No msg received...");
        println!("{:?}", m1_status);
       // assert_eq!(m1_status, Status::Ready);

        motor1.disable().await.expect("No msg received...");

        let m1_status = motor1.get_status().await.expect("No msg received...");
        println!("{:?}", m1_status);
       // assert_eq!(m1_status, Status::Disabled);

    });
    enable.await.unwrap();
    client.await.unwrap().expect("TODO: panic message");

}
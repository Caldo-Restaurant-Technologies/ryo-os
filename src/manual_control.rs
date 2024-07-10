use crate::bag_handler::{load_bag, BagHandlingCmd, ManualBagHandlingCmd};
use crate::hmi::{empty, full};
use crate::ryo::{make_gripper, make_hatches, RyoIo};
use bytes::{Buf, Bytes};
use control_components::controllers::{clear_core, ek1100_io};
use control_components::subsystems::bag_handling::BagGripper;
use http_body_util::{combinators::BoxBody, BodyExt, Empty, Full};
use hyper::{Method, Request, Response, StatusCode};
use hyper_util::rt::TokioIo;
use log::info;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::Duration;
use tokio::net::TcpListener;
use tokio::sync::mpsc::Sender;

type HTTPResult = Result<Response<BoxBody<Bytes, hyper::Error>>, hyper::Error>;
type HTTPRequest = Request<hyper::body::Incoming>;

pub enum ActuatorCmd {
    Open,
    Close,
}
pub enum ManualCmd {
    GetMotorStates,
    GetInputStates,
    Gripper(ActuatorCmd),
    LoadBag,
    DispenseBag,
    HatchCmd(ActuatorCmd),
    GantryCmd(usize),
    Dispense(usize),
    CancelDispense(usize),
}

pub async fn handle_gripper_req(body: Bytes, mut gripper: BagGripper) {
    info!("{:?}", body);
    if body.len() > 0 {
        if body[0] == b'o' {
            info!("Opening Gripper");
            gripper.open().await;
        } else if body[0] == b'c' {
            info!("Closing Gripper");
            gripper.close().await;
        }
    }
}

pub async fn handle_hatch_req(body: Bytes, io: RyoIo) {
    if body.len() >= 2 {
        let hatch_id = (body[0] -48) as usize;
        let operation = body[1];
        info!("Hatch {:}", hatch_id);
        let mut hatches = make_hatches(io.cc1, io.cc2);
        let hatch = hatches.get_mut(hatch_id);
        if hatch.is_some() {
            if operation == b'o' {
                info!("Opening Hatch {:}", hatch_id);
                hatch.unwrap().timed_open(Duration::from_secs_f64(2.1)).await;
            } else if operation == b'c' {
                info!("Closing Hatch {:}", hatch_id);
                hatch.unwrap().timed_close(Duration::from_secs_f64(2.1)).await;
            }
        }
    }
}

// pub async fn manual_request_handler(req: HTTPRequest, io: RyoIo) -> HTTPResult {
//     match (req.method(), req.uri().path()) {
//         (&Method::OPTIONS, _) => {
//             let response = Response::builder()
//                 .status(204)
//                 .header("Access-Control-Allow-Origin", "*")
//                 .header("Access-Control-Allow-Headers", "*")
//                 .header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
//                 .body(full("test"))
//                 .unwrap();
//             Ok(response)
//         }
//         (&Method::POST, "/gripper") => {
//             let body = req.collect().await?.to_bytes();
//             let gripper = make_gripper(io.cc1, io.cc2);
//             handle_gripper_req(body, gripper).await;
//             Ok(Response::new(full("Gripper request complete")))
//         }
//         (&Method::POST, "/dispense_bag") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/load_bag") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/hatch") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/gantry") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/dispense") => Ok(Response::new(full("WIP"))),
//         (&Method::POST, "/cancel") => Ok(Response::new(full("WIP"))),
// 
//         (_, _) => {
//             let mut not_found = Response::new(empty());
//             *not_found.status_mut() = StatusCode::NOT_FOUND;
//             Ok(not_found)
//         }
//     }
// }

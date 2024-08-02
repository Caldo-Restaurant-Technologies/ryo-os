use crate::config::{SEALER_MOVE_DOOR_TIME, SEALER_MOVE_TIME};
use crate::ryo::{make_sealer, make_trap_door, RyoIo};
use control_components::components::clear_core_io::HBridgeState;
use log::info;
use std::time::Duration;
use tokio::sync::mpsc::Receiver;
use tokio::time::sleep;

pub enum SealerCmd {
    Seal,
    Reset,
}
pub async fn sealer(io: RyoIo, mut rx: Receiver<SealerCmd>) {
    let mut sealer = make_sealer(io.clone());
    let mut trap_door = make_trap_door(io);
    while let Some(cmd) = rx.recv().await {
        match cmd {
            SealerCmd::Seal => {
                info!("Running seal cycle");
                // Seal
                sealer.timed_move_seal(SEALER_MOVE_TIME).await;

                // Release from sealer
                trap_door.actuate(HBridgeState::Off).await;
                trap_door.actuate(HBridgeState::Neg).await;
                sleep(SEALER_MOVE_DOOR_TIME).await;
                trap_door.actuate(HBridgeState::Off).await;
                sleep(Duration::from_millis(100)).await;
                trap_door.actuate(HBridgeState::Pos).await;
                sleep(SEALER_MOVE_DOOR_TIME).await;
                trap_door.actuate(HBridgeState::Off).await;

                info!("Bag released");
            }
            SealerCmd::Reset => {
                sealer.timed_retract_actuator(SEALER_MOVE_TIME).await;
                trap_door.actuate(HBridgeState::Pos).await;
                sleep(SEALER_MOVE_DOOR_TIME).await;
                trap_door.actuate(HBridgeState::Off).await;
                info!("Sealer reset");
            }
        }
    }
}

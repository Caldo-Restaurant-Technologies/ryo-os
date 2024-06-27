use ethercrab::{
    std::{ethercat_now, tx_rx_task},
    Client, ClientConfig, PduStorage, Timeouts,
};
use log::info;
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::Duration,
};
use tokio::sync::mpsc::error::TryRecvError;
use tokio::sync::mpsc::Receiver;
use tokio::time::MissedTickBehavior;

const MAX_SLAVES: usize = 16;
pub(crate) const MAX_PDU_DATA: usize = PduStorage::element_size(1100);
pub(crate) const MAX_FRAMES: usize = 16;
const PDI_LEN: usize = 64;
const INTERFACE: &str = "enp1s0f0";

static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

pub struct Sealer {
    open_door_byte: u8,
    close_door_byte: u8,
    apply_heater_byte: u8,
    remove_heater_byte: u8,
}
impl Sealer {
    pub fn new(
        open_door_byte: u8,
        close_door_byte: u8,
        apply_heater_byte: u8,
        remove_heater_byte: u8,
    ) -> Self {
        Self {
            open_door_byte,
            close_door_byte,
            apply_heater_byte,
            remove_heater_byte,
        }
    }

    pub async fn actor(&self, mut sealer_rx: Receiver<SealerCommand>) {
        info!("Starting ethercrab...");
        let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("Can only split once");
        let client = Arc::new(Client::new(
            pdu_loop,
            Timeouts {
                wait_loop_delay: Duration::from_millis(2),
                mailbox_response: Duration::from_millis(1000),
                ..Default::default()
            },
            ClientConfig::default(),
        ));
        tokio::spawn(tx_rx_task(INTERFACE, tx, rx).expect("Spawn TX/RX task"));
        let group = client
            .init_single_group::<MAX_SLAVES, PDI_LEN>(ethercat_now)
            .await
            .expect("Init");
        info!("Discovered {:?} slaves", group.len());
        let mut group = group.into_op(&client).await.expect("Pre-Op -> Op");
        for slave in group.iter(&client) {
            let (i, o) = slave.io_raw();
            info!(
                "Slave {:#06x} {} inputs: {} bytes, outputs: {} bytes",
                slave.configured_address(),
                slave.name(),
                i.len(),
                o.len(),
            );
        }
        let mut tick_interval = tokio::time::interval(Duration::from_millis(5));
        tick_interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let shutdown = Arc::new(AtomicBool::new(false));
        signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&shutdown))
            .expect("Register hook");

        loop {
            match sealer_rx.try_recv() {
                Ok(cmd) => {
                    let (cmd_byte, duration) = match cmd {
                        SealerCommand::OpenDoor(duration) => (self.open_door_byte, duration),
                        SealerCommand::CloseDoor(duration) => (self.close_door_byte, duration),
                        SealerCommand::ApplyHeater(duration) => (self.apply_heater_byte, duration),
                        SealerCommand::RemoveHeater(duration) => {
                            (self.remove_heater_byte, duration)
                        }
                    };
                    let init_time = tokio::time::Instant::now();
                    let delay = Duration::from_secs_f64(duration);
                    while tokio::time::Instant::now() - init_time < delay {
                        group.tx_rx(&client).await.expect("TX/RX");
                        let mut slave = group.slave(&client, 1).unwrap();
                        let (_, o) = slave.io_raw_mut();

                        if shutdown.load(Ordering::Relaxed) {
                            info!("Shutting down...");
                            break;
                        }
                        info!("thinks it's doing something ig?");
                        o[0] = cmd_byte;
                        tick_interval.tick().await;
                    }
                }
                Err(e) => match e {
                    TryRecvError::Empty => continue,
                    TryRecvError::Disconnected => break,
                },
            };
        }
        let group = group.into_safe_op(&client).await.expect("Op -> Safe-Op");
        info!("Op -> Safe-Op");
        let group = group.into_pre_op(&client).await.expect("Safe-Op -> Pre-Op");
        info!("Safe-Op -> Pre-Op");
        let _group = group.into_init(&client).await.expect("Pre-Op -> Init");
        info!("Pre-Op -> Init... Shutdown Complete!");
    }
}

pub enum SealerCommand {
    OpenDoor(f64),
    CloseDoor(f64),
    ApplyHeater(f64),
    RemoveHeater(f64),
}

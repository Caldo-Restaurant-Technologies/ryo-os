use crate::recipe_handling::Ingredient;
use crate::recipe_handling::Ingredient::Tortelloni;
use crate::ryo::{RyoRunState, RyoState};
use control_components::components::scale::ScaleCmd;
use firebase_rs::*;
use log::error;
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc::Sender;
use tokio::sync::{oneshot, Mutex};
use tokio::time;

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
struct JobProgress {
    job_progress: usize,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(rename_all = "UPPERCASE")]
pub enum NodeLevel {
    Low,
    Med,
    Loaded,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
struct NodeLevels {
    node_a_level: NodeLevel,
    node_a_level_ingredient_id: String,
    node_b_level: NodeLevel,
    node_b_level_ingredient_id: String,
    node_c_level: NodeLevel,
    node_c_level_ingredient_id: String,
    node_d_level: NodeLevel,
    node_d_level_ingredient_id: String,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
struct NodeWeights {
    node_a_weight: i32,
    node_b_weight: i32,
    node_c_weight: i32,
    node_d_weight: i32,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
enum SystemStatus {
    PauseJob,
    RunJob,
    RunningJob,
    ReadyToStartJob,
    CancelJob,
    ResumeJob,
    StopSystem,
    PreparingJob,
    RefillNodesSystem,
    Faulted,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(rename_all = "UPPERCASE")]
enum DoorStatus {
    Open,
    Close,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
#[serde(rename_all = "camelCase")]
pub struct Status {
    front_door_status: String,
    system_status: SystemStatus,
}

impl Status {
    pub async fn update_ryo_state(
        &mut self,
        mut ryo_state: RyoState,
        system_mode: Arc<Mutex<SystemMode>>,
    ) -> RyoState {
        let mode = system_mode.lock().await;
        match *mode {
            SystemMode::UI => {
                ryo_state.set_run_state(RyoRunState::UI);
            }
            SystemMode::Cycle => match self.system_status {
                SystemStatus::RunJob | SystemStatus::RunningJob => {
                    match ryo_state.get_run_state() {
                        RyoRunState::Faulted => (),
                        _ => ryo_state.set_run_state(RyoRunState::NewJob),
                    }
                }
                SystemStatus::ResumeJob => {
                    ryo_state.set_run_state(RyoRunState::Running);
                    ryo_state.clear_failures();
                }
                _ => (),
            },
            SystemMode::Maintenance => {
                // TODO: create maintenance mode
                ryo_state.set_run_state(RyoRunState::UI);
            }
            SystemMode::Clean => {
                // TODO: create cleaning mode
                ryo_state.set_run_state(RyoRunState::UI);
            }
        }

        ryo_state
    }
}

impl Default for Status {
    fn default() -> Self {
        Self {
            front_door_status: "open".to_string(),
            system_status: SystemStatus::StopSystem,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Default)]
// #[serde(rename_all = "camelCase")]
pub enum SystemMode {
    Clean,
    Maintenance,
    #[default]
    UI,
    Cycle,
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct JobOrder {
    is_single_ingredient: bool,
    node_a_ingredient: Option<Ingredient>,
    node_b_ingredient: Option<Ingredient>,
    node_c_ingredient: Option<Ingredient>,
    node_d_ingredient: Option<Ingredient>,
}
impl Default for JobOrder {
    fn default() -> Self {
        Self {
            is_single_ingredient: true,
            node_a_ingredient: Some(Tortelloni),
            node_b_ingredient: Some(Tortelloni),
            node_c_ingredient: Some(Tortelloni),
            node_d_ingredient: Some(Tortelloni),
        }
    }
}
impl JobOrder {
    pub fn from_recipe(is_single_ingredient: bool, recipe: [Option<Ingredient>; 4]) -> Self {
        Self {
            is_single_ingredient,
            node_a_ingredient: recipe[0].clone(),
            node_b_ingredient: recipe[1].clone(),
            node_c_ingredient: recipe[2].clone(),
            node_d_ingredient: recipe[3].clone(),
        }
    }

    pub fn get_ingredients(&self) -> [Option<Ingredient>; 4] {
        [
            self.node_a_ingredient.clone(),
            self.node_b_ingredient.clone(),
            self.node_c_ingredient.clone(),
            self.node_d_ingredient.clone(),
        ]
    }
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
struct SystemCleaningMaintenance {
    bag_management: String,
    clean_mode: String,
    front_door_control: String,
    side_door_control: String,
}

#[derive(Serialize, Deserialize, Debug)]
struct Response {
    name: String,
}
#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "PascalCase")]
struct LiveWeights {
    node_readings: NodeReadings,
}
#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
struct NodeReadings {
    node_a_reading: f64,
    node_b_reading: f64,
    node_c_reading: f64,
    node_d_reading: f64,
}

pub struct RyoFirebaseClient {
    firebase: Firebase,
}

impl RyoFirebaseClient {
    pub fn new() -> Self {
        let firebase = Firebase::new("https://ryo-backend-default-rtdb.firebaseio.com/")
            .unwrap()
            .at("ryo0001");
        RyoFirebaseClient { firebase }
    }

    pub async fn set_job_progress(&self, progress: usize) {
        let progress = JobProgress {
            job_progress: progress,
        };
        if let Err(e) = self.firebase.at("JobProgress").update(&progress).await {
            error!("{e}");
        }
    }

    pub async fn set_node_levels(&self, node_levels: &[NodeLevel], ingredient_ids: &[usize]) {
        let nodes = NodeLevels {
            node_a_level: node_levels[0].clone(),
            node_a_level_ingredient_id: ingredient_ids[0].to_string(),
            node_b_level: node_levels[1].clone(),
            node_b_level_ingredient_id: ingredient_ids[1].to_string(),
            node_c_level: node_levels[2].clone(),
            node_c_level_ingredient_id: ingredient_ids[2].to_string(),
            node_d_level: node_levels[3].clone(),
            node_d_level_ingredient_id: ingredient_ids[3].to_string(),
        };
        if let Err(e) = self.firebase.at("NodeLevels").update(&nodes).await {
            error!("{e}");
        }
    }

    async fn set_weight_readings(&self, weights: &[f64]) {
        let readings = NodeReadings {
            node_a_reading: weights[0],
            node_b_reading: weights[1],
            node_c_reading: weights[2],
            node_d_reading: weights[3],
        };
        let live_weights = LiveWeights {
            node_readings: readings,
        };

        if let Err(e) = self.firebase.update(&live_weights).await {
            error!("{e}")
        }
    }

    pub async fn set_job_order(&self, job_order: JobOrder) {
        if let Err(e) = self.firebase.at("JobOrder").update(&job_order).await {
            error!("{e}");
        }
    }

    pub async fn update(
        &mut self,
        scale_senders: &[Sender<ScaleCmd>],
        state: Arc<Mutex<Status>>,
        system_mode: Arc<Mutex<SystemMode>>,
        shutdown: Arc<AtomicBool>,
    ) {
        let mut interval = time::interval(Duration::from_millis(500));
        loop {
            if shutdown.load(Ordering::Relaxed) {
                break;
            }
            let mut weights = Vec::with_capacity(scale_senders.len());
            for sender in scale_senders {
                let (tx_resp, rx_resp) = oneshot::channel::<f64>();
                sender
                    .send(ScaleCmd(tx_resp))
                    .await
                    .expect("Failed to send ScaleCmd");
                let weight = rx_resp.await.expect("Failed to unwrap weight");
                weights.push(weight);
            }
            //self.set_weight_readings(weights.as_slice()).await;

            if let Ok(status) = self.firebase.at("Status").get::<Status>().await {
                let mut state = state.lock().await;
                *state = status;
            } else {
                error!("Failed to get status from firebase");
            }

            if let Ok(mode) = self.firebase.at("SystemMode").get::<SystemMode>().await {
                let mut system_mode = system_mode.lock().await;
                *system_mode = mode;
            } else {
                error!("Failed to get mode from firebase");
            }

            interval.tick().await;
        }
    }
}

use std::collections::HashMap;
use serde::{Deserialize, Serialize};



#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Ingredient {
    id: String,
    #[serde(rename = "type")]
    ingredient_type: String,
    name: String,
    pub weight: f64,
    unit: String,
    #[serde(rename = "motor-speed")]
    pub motor_speed: f64,
    #[serde(rename = "sample-rate")]
    pub sample_rate: f64,
    #[serde(rename = "cutoff-frequency")]
    pub cutoff_frequency: f64,
    #[serde(rename = "check-offset")]
    pub check_offset: f64,
    #[serde(rename = "stop-offset")]
    pub stop_offset: f64,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct Recipe {
    id: String,
    name: String,
    pub ingredients: Vec<Ingredient>
}

pub fn get_sample_recipe() -> HashMap<String, Recipe> {
    let json_str = r#"[
        {
            "id": "0",
            "name": "Pad Thai (Regular)",
            "ingredients": [
                {
                    "id": "7",
                    "type": "regular",
                    "name": "Rice Noodles",
                    "weight": 8.0,
                    "unit": "oz",
                    "motor-speed": 0.2,
                    "sample-rate": 50,
                    "cutoff-frequency": 0.5,
                    "check-offset": 15.0,
                    "stop-offset": 7.0
                },
                {
                    "id": "6",
                    "type": "regular",
                    "name": "Cabbage",
                    "weight": 1.0,
                    "unit": "oz",
                    "motor-speed": 0.2,
                    "sample-rate": 50,
                    "cutoff-frequency": 0.5,
                    "check-offset": 15.0,
                    "stop-offset": 7.0
                },
                {
                    "id": "7",
                    "type": "regular",
                    "name": "Rice Noodles",
                    "weight": 8.0,
                    "unit": "oz",
                    "motor-speed": 0.2,
                    "sample-rate": 50,
                    "cutoff-frequency": 0.5,
                    "check-offset": 15.0,
                    "stop-offset": 7.0
                },
                {
                    "id": "6",
                    "type": "regular",
                    "name": "Cabbage",
                    "weight": 1.0,
                    "unit": "oz",
                    "motor-speed": 0.2,
                    "sample-rate": 50,
                    "cutoff-frequency": 0.5,
                    "check-offset": 15.0,
                    "stop-offset": 7.0
                }
            ]
        }
    ]"#;

    let recipes: Vec<Recipe> = serde_json::from_str(json_str).unwrap();
    
    recipes
        .into_iter()
        .map(|recipe|{ (recipe.id.clone(), recipe)}).collect()
}
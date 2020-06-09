use serde::Deserialize;
use std::error::Error;

#[derive(Deserialize, Debug, Clone)]
pub struct Vector {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Deserialize, Debug, Clone)]
pub struct Quaternion {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
    pub is_identity: bool,
}

#[derive(Deserialize, Debug, Clone)]
pub struct Pose {
    pub device_index: i32,
    pub device_class: String,
    pub position: Vector,
    pub rotation: Quaternion,
}

impl Pose {
    pub fn deserialize(data: &[u8]) -> Result<Pose, Box<dyn Error>> {
        Ok(serde_json::from_slice::<Pose>(data)?)
    }
}

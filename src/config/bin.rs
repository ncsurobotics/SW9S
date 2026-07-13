//! Configuration for the bin mission

use serde::{Deserialize, Serialize};

/// Bin mission config submodule
#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    pub depth: f32,
    pub speed: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            depth: -1.25,
            speed: 0.2,
        }
    }
}

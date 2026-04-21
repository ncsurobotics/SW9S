//! Configuration for the octagon family of missions

use serde::{Deserialize, Serialize};

/// Octagon mission config submodule
#[derive(Debug, Serialize, Deserialize)]
pub struct Config {}

impl Default for Config {
    fn default() -> Self {
        Self {}
    }
}

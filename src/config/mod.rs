//! External configuration, parsable from a config file

pub mod bin;
pub mod coinflip;
pub mod gate;
pub mod octagon;
pub mod path_align;
pub mod slalom;
pub mod sonar;
pub mod spin;

use std::fs::read_to_string;

use crate::vision::Yuv;
use color_eyre::eyre::Result;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::ops::RangeInclusive;

/// Time to wait on graceful shutdown before hard shutdown
pub const SHUTDOWN_TIMEOUT: u64 = 5;

// Default values
const CONFIG_FILE: &str = "config.toml";
const CONTROL_BOARD_PATH: &str = "/dev/ttyACM0";
const CONTROL_BOARD_BACKUP_PATH: &str = "/dev/ttyACM3";
const MEB_PATH: &str = "/dev/ttyACM2";
const FRONT_CAM: &str = "/dev/video0";
const BOTTOM_CAM: &str = "/dev/video1";
const ZED_NAMESPACE: &str = "/zed/";
const ZED_DEPTH_TOPIC: &str = "depth/depth_registered";
const ZED_CLOUD_TOPIC: &str = "point_cloud/cloud_registered";
const ZED_POSE_TOPIC: &str = "pose";

/// Top-level configuration
#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    /// The path to the control board serial port
    pub control_board_path: String,
    /// The backup path to the control board serial port
    pub control_board_backup_path: String,
    /// The path to the main electronics board serial port
    pub meb_path: String,
    /// The path to the front camera video device
    pub front_cam_path: String,
    /// The path to the bottom camera video device
    pub bottom_cam_path: String,
    /// The active color profile used for vision. Must be in `color_profiles`
    pub color_profile: String,
    /// The available color profiles usable for vision
    pub color_profiles: HashMap<String, ColorProfile>,
    /// The [`Side`] the shark prop is on
    pub shark: Side,
    /// The [`Side`] the saw fish prop is on
    pub saw_fish: Side,
    /// BlueRobotics Ping360 sonar config submodule
    pub sonar: sonar::Config,
    /// Missions config submodule
    pub missions: Missions,
    /// ZED ROS2 subsystem config submodule
    pub zed_ros2: ZedRos2Config,
}

impl Config {
    /// Generates a populated [`Config`] from a `config.toml` file in the current directory.
    ///
    /// # Errors
    ///
    /// This function will return an error if `config.toml` does not exist.
    pub fn new() -> Result<Self> {
        let config_string = read_to_string(CONFIG_FILE)?;
        Ok(toml::from_str(&config_string)?)
    }
}

impl Config {
    /// Returns the configured [`ColorProfile`] if it exists in `color_profiles`.
    pub fn get_color_profile(&self) -> Option<&ColorProfile> {
        self.color_profiles.get(&self.color_profile)
    }
}

impl Default for Config {
    fn default() -> Self {
        Self {
            control_board_path: CONTROL_BOARD_PATH.to_string(),
            control_board_backup_path: CONTROL_BOARD_BACKUP_PATH.to_string(),
            meb_path: MEB_PATH.to_string(),
            front_cam_path: FRONT_CAM.to_string(),
            bottom_cam_path: BOTTOM_CAM.to_string(),
            sonar: sonar::Config::default(),
            missions: Missions::default(),
            color_profile: "".to_string(),
            color_profiles: HashMap::new(),
            shark: Side::default(),
            saw_fish: Side::default(),
            zed_ros2: ZedRos2Config::default(),
        }
    }
}

/// ZED ROS2 subsystem config submodule.
#[derive(Debug, Serialize, Deserialize)]
pub struct ZedRos2Config {
    pub namespace: String,
    pub depth_topic: String,
    pub cloud_topic: String,
    pub pose_topic: String,
}

impl Default for ZedRos2Config {
    fn default() -> Self {
        Self {
            namespace: ZED_NAMESPACE.to_string(),
            depth_topic: ZED_DEPTH_TOPIC.to_string(),
            cloud_topic: ZED_CLOUD_TOPIC.to_string(),
            pose_topic: ZED_POSE_TOPIC.to_string(),
        }
    }
}

/// Missions config submodule.
///
/// Each submodule corresponds to a [mission](crate::missions).
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct Missions {
    pub gate: gate::Config,
    pub path_align: path_align::Config,
    pub slalom: slalom::Config,
    pub bin: bin::Config,
    pub octagon: octagon::Config,
    pub coinflip: coinflip::Config,
    pub spin: spin::Config,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ColorProfile {
    pub red: RangeInclusive<Yuv>,
    pub orange: RangeInclusive<Yuv>,
    pub yellow: RangeInclusive<Yuv>,
    pub purple: RangeInclusive<Yuv>,
    pub black: RangeInclusive<Yuv>,
}

#[derive(Debug, Serialize, Deserialize, PartialEq, Eq, Hash, Clone)]
pub enum Side {
    Right,
    Left,
}

impl Default for Side {
    fn default() -> Self {
        Self::Right
    }
}

use itertools::Itertools;
use tokio::io::WriteHalf;
use tokio::time::{sleep, Duration};
use tokio_serial::SerialStream;

use crate::{
    config::{gate::Config, ColorProfile, Side},
};

use super::{
    action::ActionExec,
    action_context::{GetControlBoard, GetMainElectronicsBoard},
    basic::DelayAction,
};

pub struct Robot {
    initial_yaw: f32,
    cb: &ControlBoard,
    config: &Config,
}

impl Robot {
    pub fn getYaw(&mut self) -> f32{
        let yaw = loop {
            if let Some(initial_angle) = self.cb.responses().get_angles().await {
                break *initial_angle.yaw();
            } else {
                #[cfg(feature = "logging")]
                logln!("Failed to get initial angle");
            }
        };
        self.initial_yaw = yaw;
    }

    pub fn stb2fdTime(&mut self) {
        let _ = self.cb.stability_2_speed_set(0.0, self.config.speed, 0.0, 0.0, self.initial_yaw, self.config.depth).await;

        sleep(Duration::from_secs_f32(self.config.traversal_duration)).await;

        let _ = self.cb.stability_1_speed_set(0.0, 0.0, 0.0, 0.0, 0.0, self.config.depth).await;
    }

    pub fn initial_yaw(&self) -> f32 {
        self.initial_yaw
    }
}
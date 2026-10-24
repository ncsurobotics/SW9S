//! Dispatches named missions and handles cancellation during execution.

use crate::{
    comms::{control_board, local_speed_set},
    config::{MissionsConfig, MotorTestMissionConfig, TestMissionConfig},
    logging::{bail, error, eyre, info, Result},
};
use std::{future::Future, pin::Pin, time::Duration, u64};

use tokio::time::sleep;
use tokio_util::sync::CancellationToken;

/// Runs a named mission, returning an error for unknown names or cancellation.
/// Runs the named mission until completion or shutdown.
#[tracing::instrument(skip(shutdown_token))]
pub async fn run_mission(
    name: &str,
    config: MissionsConfig,
    shutdown_token: CancellationToken,
) -> Result<()> {
    let mission: Pin<Box<dyn Future<Output = Result<()>> + Send>> = match name {
        "sleep" => Box::pin(sleep_mission()),
        "test" => Box::pin(test_mission(config.test)),
        "slow-spin" => Box::pin(slow_spin_mission()),
        "motor-x-right" => Box::pin(motor_x_right(config.motor_test)),
        "motor-x-left" => Box::pin(motor_x_left(config.motor_test)),
        "motor-y-forward" => Box::pin(motor_y_forward(config.motor_test)),
        "motor-y-back" => Box::pin(motor_y_back(config.motor_test)),
        "motor-z-up" => Box::pin(motor_z_up(config.motor_test)),
        "motor-z-down" => Box::pin(motor_z_down(config.motor_test)),
        "motor-pitch-positive" => Box::pin(motor_pitch_positive(config.motor_test)),
        "motor-pitch-negative" => Box::pin(motor_pitch_negative(config.motor_test)),
        "motor-roll-positive" => Box::pin(motor_roll_positive(config.motor_test)),
        "motor-roll-negative" => Box::pin(motor_roll_negative(config.motor_test)),
        "motor-yaw-cw" | "motor-yaw-positive" => Box::pin(motor_yaw_positive(config.motor_test)),
        "motor-yaw-ccw" | "motor-yaw-negative" => Box::pin(motor_yaw_negative(config.motor_test)),
        _ => {
            error!("Unknown mission: {name}");
            bail!("Unknown mission: {name}")
        }
    };

    info!("Running {name}");
    let result = match shutdown_token.run_until_cancelled(mission).await {
        Some(r) => {
            info!("Ran {name}");
            r
        }
        None => {
            error!("Mission {name} cancelled by shutdown");
            Err(eyre!("Mission {name} cancelled by shutdown"))
        }
    };
    if name.starts_with("motor-") {
        // This is outside the cancelled future, so cancellation cannot skip the stop.
        local_speed_set([0.0; 6]).await?;
    }
    result
}

/// Runs one local degree of freedom; the dispatcher sends the final zero command.
async fn motor_test(config: MotorTestMissionConfig, axis: usize, direction: f32) -> Result<()> {
    if !config.speed.is_finite() || config.speed <= 0.0 || config.speed > 1.0 {
        bail!("missions.motor_test.speed must be greater than 0 and at most 1");
    }
    let mut speeds = [0.0; 6];
    speeds[axis] = direction * config.speed;
    info!(?speeds, "Testing local motor motion until shutdown");
    local_speed_set(speeds).await?;
    std::future::pending::<Result<()>>().await
}

async fn motor_x_right(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 0, 1.0).await
}

async fn motor_x_left(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 0, -1.0).await
}

async fn motor_y_forward(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 1, 1.0).await
}

async fn motor_y_back(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 1, -1.0).await
}

async fn motor_z_up(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 2, 1.0).await
}

async fn motor_z_down(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 2, -1.0).await
}

async fn motor_pitch_positive(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 3, 1.0).await
}

async fn motor_pitch_negative(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 3, -1.0).await
}

async fn motor_roll_positive(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 4, 1.0).await
}

async fn motor_roll_negative(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 4, -1.0).await
}

async fn motor_yaw_positive(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 5, 1.0).await
}

async fn motor_yaw_negative(config: MotorTestMissionConfig) -> Result<()> {
    motor_test(config, 5, -1.0).await
}

async fn test_mission(config: TestMissionConfig) -> Result<()> {
    let cb = control_board().await?;

    info!("Descending to test depth");
    cb.stability_2_speed_set(0.0, 0.0, 0.0, 0.0, 0.0, config.depth_meters)
        .await?;
    sleep(Duration::from_secs(config.down_seconds)).await;

    info!("Moving forward at test depth");
    cb.stability_2_speed_set(
        0.0,
        config.forward_speed,
        0.0,
        0.0,
        0.0,
        config.depth_meters,
    )
    .await?;
    sleep(Duration::from_secs(config.forward_seconds)).await;

    cb.stability_2_speed_set(0.0, 0.0, 0.0, 0.0, 0.0, config.depth_meters)
        .await?;
    Ok(())
}
async fn slow_spin_mission() -> Result<()> {
    let cb = control_board().await?;

    info!("Doing slow spin");
    cb.raw_speed_set([0.1; 8]).await?;
    sleep(Duration::from_secs(u64::MAX)).await;
    Ok(())
}

async fn sleep_mission() -> Result<()> {
    sleep(Duration::from_secs(5)).await;
    Ok(())
}

//! Dispatches named missions and handles cancellation during execution.

use crate::{
    comms::control_board,
    config::TestMissionConfig,
    logging::{bail, error, eyre, info, Result},
};
use std::{future::Future, pin::Pin, time::Duration};

use tokio::time::sleep;
use tokio_util::sync::CancellationToken;

/// Runs a named mission, returning an error for unknown names or cancellation.
/// Runs the named mission until completion or shutdown.
#[tracing::instrument(skip(shutdown_token))]
pub async fn run_mission(
    name: &str,
    test_config: TestMissionConfig,
    shutdown_token: CancellationToken,
) -> Result<()> {
    let mission: Pin<Box<dyn Future<Output = Result<()>> + Send>> = match name {
        "sleep" => Box::pin(sleep_mission()),
        "test" => Box::pin(test_mission(test_config)),
        _ => {
            error!("Unknown mission: {name}");
            bail!("Unknown mission: {name}")
        }
    };

    info!("Running {name}");
    match shutdown_token.run_until_cancelled(mission).await {
        Some(r) => {
            info!("Ran {name}");
            r
        }
        None => {
            error!("Mission {name} cancelled by shutdown");
            Err(eyre!("Mission {name} cancelled by shutdown"))
        }
    }
}

async fn test_mission(config: TestMissionConfig) -> Result<()> {
    let cb = control_board().await?;

    info!("Descending to test depth");
    cb.stability_2_speed_set(0.0, 0.0, 0.0, 0.0, 0.0, config.depth_meters)
        .await?;
    sleep(Duration::from_secs(config.down_seconds)).await;

    info!("Moving forward at test depth");
    cb.stability_2_speed_set(
        config.forward_speed,
        0.0,
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

async fn sleep_mission() -> Result<()> {
    sleep(Duration::from_secs(5)).await;
    Ok(())
}

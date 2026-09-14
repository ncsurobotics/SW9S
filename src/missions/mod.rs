//! Dispatches named missions and handles cancellation during execution.

use crate::logging::{bail, error, eyre, info, Result};
use std::time::Duration;

use tokio::time::sleep;
use tokio_util::sync::CancellationToken;

/// Runs a named mission, returning an error for unknown names or cancellation.
/// The gate mission currently waits five seconds as a placeholder.
#[tracing::instrument(skip(shutdown_token))]
pub async fn run_mission(name: &str, shutdown_token: CancellationToken) -> Result<()> {
    let mission = match name {
        "gate" => sleep(Duration::from_secs(5)),
        _ => {
            error!("Unknown mission: {name}");
            bail!("Unknown mission: {name}")
        }
    };

    info!("Running {name}");
    match shutdown_token.run_until_cancelled(mission).await {
        Some(r) => {
            info!("Ran {name}");
            Ok(r)
        }
        None => {
            error!("Mission {name} cancelled by shutdown");
            Err(eyre!("Mission {name} cancelled by shutdown"))
        }
    }
}

use crate::logging::{bail, error, eyre, info, Result};
use std::time::Duration;

use tokio::time::sleep;
use tokio_util::sync::CancellationToken;

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

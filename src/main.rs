use sw9s::{
    cli::{Args, Parser},
    config::Config,
    logging::{self, error, info, instrument, Result},
    missions::run_mission,
};

use std::time::Duration;
use tokio::{spawn, time::sleep};
use tokio_util::sync::CancellationToken;

#[instrument]
#[tokio::main]
async fn main() -> Result<()> {
    logging::install()?;
    let args = Args::parse();
    let config = Config::new()?;
    info!("{:#?}", config);

    let shutdown_token = CancellationToken::new();
    let shutdown_handler_task = spawn(shutdown_handler(shutdown_token.clone()));
    let arm_handler_task = spawn(arm_handler(shutdown_token.clone()));
    let mut mission_result = Ok(());

    for mission in args.missions {
        mission_result = run_mission(&mission, shutdown_token.clone()).await;
        if mission_result.is_err() {
            error!("Halting mission execution");
            break;
        }
    }
    if !shutdown_token.is_cancelled() {
        info!("Mission execution finished, signaling shutdown");
        shutdown_token.cancel();
    } else {
        info!("Mission execution finished, but shutdown has already begun");
    }

    arm_handler_task.await??;
    shutdown_handler_task.await??;
    mission_result
}

/// Stops thrusters and resets manipulators upon recieving shutdown signal
#[instrument(skip_all)]
async fn shutdown_handler(shutdown_token: CancellationToken) -> Result<()> {
    shutdown_token.cancelled().await;
    info!("Begining shutdown sequence");
    // Log out cb sensor status
    info!("Control Board Status: Healthy");
    // Shutdown motors
    info!("Stopping thrusters");
    info!("Thrusters stopped");
    // Reset torpedos (but not actually because it never completes)
    info!("Reseting torpedos");
    info!("Torpedos reset");
    sleep(Duration::from_secs(5)).await; // All of the above takes 5 seconds for mockup purposes
    info!("Finished shutdown sequence");
    Ok(())
}

/// Sends shutdown signal once robot is armed and subsequently disarmed
#[instrument(skip_all)]
async fn arm_handler(shutdown_token: CancellationToken) -> Result<()> {
    shutdown_token
        .run_until_cancelled(async {
            info!("Waiting for hardware arm");
            sleep(Duration::from_secs(5)).await;
            info!("Got hardware arm, waiting for disarm");
            sleep(Duration::from_secs(5)).await;
            info!("Got disarm, shutting down");
        })
        .await;
    shutdown_token.cancel();
    Ok(())
}

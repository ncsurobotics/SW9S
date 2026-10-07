//! Runs SeaWolf 9 missions and provides configuration commands.
//!
//! # Quickstart
//!
//! ```sh
//! sw9s cfg generate
//! $EDITOR config.toml
//! sw9s cfg check
//! sw9s run gate slalom bin
//! ```

mod cli;
mod comms;
mod config;
mod logging;
mod missions;

use crate::{
    cli::{CfgSubcmd, Cli, Parser, RunArgs, Subcmd},
    comms::control_board,
    config::Config,
    logging::{debug, error, info, instrument, Result},
    missions::{run_mission, SeaWolfState},
};

use bonsai_bt::{
    Behavior::{Action, After, Race, Sequence, While},
    Timer, BT,
};
use std::{collections::HashMap, time::Duration};
use tokio::{spawn, time::sleep};
use tokio_util::sync::CancellationToken;

/// Initializes logging and dispatches the selected command.
#[instrument]
#[tokio::main]
async fn main() -> Result<()> {
    logging::install()?;
    let args = Cli::parse();
    match args.subcmd {
        Subcmd::Run(args) => run(args).await,
        Subcmd::Cfg(subcmd) => match subcmd {
            CfgSubcmd::Check { file } => Config::check(&file),
            CfgSubcmd::Generate { file, force } => Config::save_default(&file, force),
        },
    }
}

/// Runs missions in order and waits for the arm and shutdown handlers to finish.
#[instrument(skip_all)]
async fn run(args: RunArgs) -> Result<()> {
    let config = Config::new(&args.config)?;
    debug!("{:#?}", config);

    let shutdown_token = CancellationToken::new();
    let shutdown_handler_task = spawn(shutdown_handler(shutdown_token.clone()));
    let arm_handler_task = spawn(arm_handler(shutdown_token.clone()));
    let mut mission_result = Ok(());

    let mut state = SeaWolfState { see_goal: None };

    let mut timer = Timer::init_time();

    for mission in args.missions {
        let blackboard: HashMap<String, i32> = HashMap::new();
        let mut behavior = Action(missions::mission_logic::gate);
        mission_result = run_mission(
            &mission,
            &mut state,
            &mut timer,
            &mut BT::new(behavior, blackboard),
        )
        .await;
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
    let cb = control_board().await?;
    shutdown_token.cancelled().await;

    info!("Begining shutdown sequence");
    // Log out cb sensor status
    info!(
        "Control Board Sensor Status: {:#?}",
        cb.sensor_status_query().await.unwrap()
    );

    // Shutdown motors
    info!("Stopping thrusters");
    cb.relative_dof_speed_set_batch(&[0.0; 6]).await?;
    info!("Thrusters stopped");

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

//! Provides shared access to the serial control board.

use crate::{
    cli::{Cli, Parser, RunArgs, Subcmd},
    config::Config,
    logging::Result,
};
use auv_control_board::SerialControlBoard;
use tokio::sync::OnceCell;

/// Control board connection retained after successful initialization.
static CONTROL_BOARD: OnceCell<SerialControlBoard> = OnceCell::const_new();

/// Sends LOCAL speeds in protocol order: x, y, z, x rotation, y rotation, z rotation.
/// The pinned control board library exposes the transport but has no LOCAL helper.
pub async fn local_speed_set(speeds: [f32; 6]) -> Result<()> {
    let cb = control_board().await?;
    let mut message = Vec::with_capacity(29);
    message.extend_from_slice(b"LOCAL");
    for speed in speeds {
        message.extend_from_slice(&speed.to_le_bytes());
    }
    cb.write_out_basic(message).await?;
    Ok(())
}

/// Returns the shared control board, initializing it from the run configuration if needed.
///
/// Returns an error if configuration loading or board initialization fails.
/// Panics if initialization is requested outside the run command.
pub async fn control_board() -> Result<&'static SerialControlBoard> {
    let args = Cli::parse();
    CONTROL_BOARD
        .get_or_try_init(|| async {
            if let Subcmd::Run(RunArgs { config, .. }) = args.subcmd {
                let config = Config::new(&config)?;
                let cb = SerialControlBoard::new(&config.cb_path, config.vehicle).await;
                Ok(cb?)
            } else {
                panic!("Tried getting config outside of run context");
            }
        })
        .await
}

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
                    let cb = SerialControlBoard::new("/dev/serial/by-id/usb-STMicroelectronics_Control_Board_v2__Virtual_COM_Port__36313632303251010061003C-if00", config.vehicle)
                        .await;
                    Ok(cb?)
                } else {
                    panic!("Tried getting config outside of run context");
                }
        }).await
}

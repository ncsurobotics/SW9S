use crate::{
    cli::{Cli, Parser, RunArgs, Subcmd},
    config::Config,
    logging::Result,
};
use auv_control_board::SerialControlBoard;
use tokio::sync::OnceCell;

static CONTROL_BOARD: OnceCell<SerialControlBoard> = OnceCell::const_new();

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

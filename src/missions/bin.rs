use tokio::io::WriteHalf;
use tokio_serial::SerialStream;

use super::action_context::{BottomCamIO, GetControlBoard};

pub async fn bin<
    Con: Send + Sync + GetControlBoard<WriteHalf<SerialStream>> + BottomCamIO,
>(
    context: &Con,
) {
    #[cfg(feature = "logging")]
    logln!("Starting bin");

    let cb = context.get_control_board();
    let _ = cb.bno055_periodic_read(true).await;
    #[cfg(feature = "logging")]
    logln!("Finished bin");
}

use bonsai_bt::Status;
use std::sync::mpsc::Sender;
use std::thread::sleep;
use std::time::Duration;

pub async fn gate_task(tx: Sender<Status>) {
    // Add Functionality for gate
    // Success only indicates shark seen and does not indicate gate fully gone through
    loop {
        sleep(Duration::from_secs(5));
        break;
    }
    tx.send(Status::Success).unwrap();
}

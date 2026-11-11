use crate::comms::control_board;
use crate::missions::Target;
use bonsai_bt::Status;
use std::sync::mpsc::Sender;
use std::time::Duration;

use crate::missions::visions_options::lookfor;

pub async fn swim_task(tx: Sender<Status>, target: Target, direction: f32) {
    // Add Functionality for swim
    // Success only indicates shark seen and does not indicate swim fully completed
    let _ = control_board()
        .await
        .unwrap()
        .stability_2_speed_set(0.0, 1.0 * direction, 0.0, 0.0, 0.0, 0.0)
        .await;
    while !lookfor(&target) {
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    tx.send(Status::Success).unwrap();
}

pub async fn face_task(tx: Sender<Status>, target: Target) {
    // Add Functionality for face
    // Success only indicates shark seen and does not indicate face fully completed
    let cd = control_board().await.unwrap();
    let _ = cd.stability_1_speed_set(0.0, 0.0, 1.0, 0.0, 0.0, 0.0).await;
    while !lookfor(&target) {
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    tx.send(Status::Success).unwrap();
}

pub async fn style_task(tx: Sender<Status>) {
    // Add Functionality for style
    // Success only indicates shark seen and does not indicate style fully completed
    let cd = control_board().await.unwrap();
    let _ = cd
        .relative_dof_speed_set(0.0, 1.0, 0.0, 1.0, 0.0, 0.0)
        .await;
    while !lookfor(&Target::Arrow) {
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    tx.send(Status::Success).unwrap();
}

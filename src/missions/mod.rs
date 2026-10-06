//! Dispatches named missions and handles cancellation during execution.

use crate::{
    logging::{bail, error, eyre, info, Result},
    missions::tasks::gate_task,
};
use bonsai_bt::{
    Event,
    Status::{self, Failure, Success},
    Timer, UpdateArgs, BT, RUNNING,
};
use std::collections::HashMap;
use std::result::Result::Ok;
use std::sync::mpsc::{channel, Receiver};
mod tasks;

use tokio_util::sync::CancellationToken;

// States sea wolf can be in
#[derive(Debug)]
pub struct SeaWolfState {
    pub see_sharks: Option<Receiver<Status>>,
}

// Advances the mission behavior tree by one tick.
// IDK how to use showdown token thing so for now just skip it
pub async fn run_mission(
    name: &str,
    state: &mut SeaWolfState,
    timer: &mut Timer,
    bt: &mut BT<&str, HashMap<String, i32>>,
) -> Result<()> {
    // have bt advance dt seconds into the future
    let dt = timer.get_dt();

    // proceed to next iteration in event loop
    let e: Event = UpdateArgs { dt }.into();

    // Update behavior tree
    #[rustfmt::skip]
     bt.tick(&e,&mut |args: bonsai_bt::ActionArgs<Event, &str>, _|
        match *args.action {
            "gate" => {
                let gate_state = &state.see_sharks;
                if let Some(gate_status) = gate_state {
                    match gate_status.recv() {
                        Ok(status) => {
                            match status {
                                Success => {
                                    // Add action if see sharks preferably turns see_sharks to none
                                    (Status::Success, args.dt)
                                },
                                Failure => {
                                    // Add action if see sharks
                                    state.see_sharks = None;
                                    (Status::Failure, args.dt)
                                },
                                Status::Running => RUNNING,
                            }
                        },
                        Err(_) => {
                            state.see_sharks = None;
                            (Status::Failure, args.dt)
                        },
                    }
                } else {
                    let (tx, rx) = channel();
                    gate_task(tx);
                    state.see_sharks = Some(rx);
                    let receiver = state.see_sharks.as_ref().unwrap();
                    let status = receiver.recv().unwrap();
                    (status, args.dt)
                }
            }
            _ => {
                error!("Unknown mission: {name}");
                (Status::Failure, args.dt)
        }
        }
     );

    info!("Running {name}");
    Ok(())
}

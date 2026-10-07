//! Dispatches named missions and handles cancellation during execution.

use crate::{
    logging::{bail, error, eyre, info, Result},
    missions::tasks::gate_task,
};
use bonsai_bt::{
    Behavior::{Action, After, Race, Sequence, While},
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
    pub see_goal: Option<Receiver<Status>>,
}

// Enum for actions by sea wolf in water
#[derive(Clone, Debug, PartialEq)]
pub enum mission_logic {
    face_towards,
    is_close,
    swim,
    style,
    gate,
}

// Advances the mission behavior tree by one tick.
// IDK how to use showdown token thing so for now just skip it
pub async fn run_mission(
    name: &str,
    state: &mut SeaWolfState,
    timer: &mut Timer,
    bt: &mut BT<mission_logic, HashMap<String, i32>>,
) -> Result<()> {
    let face_towards = Action(mission_logic::face_towards);
    let is_close = Action(mission_logic::is_close);
    let swim = Action(mission_logic::swim);
    let style = Action(mission_logic::style);

    let move_to = Sequence(vec![
        Action(face_towards),
        Race(vec![Action(swim), Action(is_close)]),
    ]);

    // have bt advance dt seconds into the future
    let dt = timer.get_dt();

    // proceed to next iteration in event loop
    let e: Event = UpdateArgs { dt }.into();

    // Update behavior tree
    #[rustfmt::skip]
     bt.tick(&e,&mut |args: bonsai_bt::ActionArgs<Event, mission_logic>, _|
        match *args.action {
            mission_logic::face_towards => {
                // Add action to face towards goal
                (Status::Success, args.dt)
            },
            mission_logic::swim => {
                // Add action to swim
                (Status::Success, args.dt)
            },
            mission_logic::style => {
                // Add action to style
                (Status::Success, args.dt)
            }
            mission_logic::gate => {
                let gate_state: &Option<Receiver<Status>> = &state.see_goal;
                if let Some(gate_status) = gate_state {
                    match gate_status.recv() {
                        Ok(status) => {
                            match status {
                                Success => {
                                    // Add action if see sharks, preferably turns see_sharks to none
                                    (Status::Success, args.dt)
                                },
                                Failure => {
                                    // Add action if see sharks
                                    state.see_goal = None;
                                    (Status::Failure, args.dt)
                                },
                                Status::Running => RUNNING,
                            }
                        },
                        Err(_) => {
                            state.see_goal = None;
                            (Status::Failure, args.dt)
                        },
                    }
                } else {
                    let (tx, rx) = channel();
                    gate_task(tx);
                    state.see_goal = Some(rx);
                    let receiver = state.see_goal.as_ref().unwrap();
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

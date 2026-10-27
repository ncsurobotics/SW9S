//! Dispatches named missions and handles cancellation during execution.

use crate::{
    logging::{bail, error, eyre, info, Result},
    missions::tasks::gate_task,
};
use bonsai_bt::{
    Behavior::{Action, After, Race, Sequence, Timeout, While},
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

#[derive(Clone, Debug, PartialEq)]
// Enum of targets to get close to or look at
pub enum Target {
    None,
    Gate,
    Shark,
}

// Enum for actions by sea wolf in water
#[derive(Clone, Debug, PartialEq)]
pub enum mission_logic {
    face_towards(Target),
    swim(Target),
}

// Advances the mission behavior tree by one tick.
// IDK how to use showdown token thing so for now just skip it
pub async fn run_mission(
    name: &str,
    state: &mut SeaWolfState,
    timer: &mut Timer,
    bt: &mut BT<mission_logic, HashMap<String, i32>>,
) -> Result<()> {
    // have bt advance dt seconds into the future
    let dt = timer.get_dt();

    // proceed to next iteration in event loop
    let e: Event = UpdateArgs { dt }.into();

    // Update behavior tree
    #[rustfmt::skip]
     bt.tick(&e,&mut |args: bonsai_bt::ActionArgs<Event, mission_logic>, _|
        match args.action {
            mission_logic::face_towards(target) => {
                // Add action to face towards goal
                let mut goal: bool = false;
                while(!goal) {
                    // Rotate robot
                    // based on targets update goal
                    // Alternative Option is having seegoal as part of sea wolf state
                    // Then we would have updating see wolf state be its own mission logic
                }
                (Status::Success, args.dt)
            },
            mission_logic::swim(target) => {
                // Alternative to face towards listed below
                // Add action to swim
                let gate_state: &Option<Receiver<Status>> = &state.see_goal;
                if let Some(gate_status) = gate_state {
                    match gate_status.recv() {
                        Ok(status) => {
                            match status {
                                Success => {
                                    // do nothing just say success
                                    (Status::Success, args.dt)
                                },
                                Failure => {
                                    // back up
                                    // rotate until see target
                                    // rerun swim
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

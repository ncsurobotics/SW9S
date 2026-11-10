//! Dispatches named missions and handles cancellation during execution.

use crate::{
    logging::{info, Result},
    missions::tasks::{face_task, swim_task},
};
use bonsai_bt::{
    Event,
    Status::{self},
    Timer, UpdateArgs, BT, RUNNING,
};
use std::collections::HashMap;
use std::result::Result::Ok;
use std::sync::mpsc::{channel, Receiver};
mod tasks;
mod visions_options;

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
pub enum MissionLogic {
    FaceTowards(Target),
    Swim(Target),
}

// Advances the mission behavior tree by one tick.
// IDK how to use showdown token thing so for now just skip it
pub async fn run_mission(
    state: &mut SeaWolfState,
    timer: &mut Timer,
    bt: &mut BT<MissionLogic, HashMap<String, i32>>,
) -> Result<()> {
    // have bt advance dt seconds into the future
    let dt = timer.get_dt();

    // proceed to next iteration in event loop
    let e: Event = UpdateArgs { dt }.into();

    // Update behavior tree
    #[rustfmt::skip]
     bt.tick(&e,&mut |args: bonsai_bt::ActionArgs<Event, MissionLogic>, _|
        match args.action {
            MissionLogic::FaceTowards(target) => {
                 // Alternative to face towards listed below
                // Add action to swim
                let face_state: &Option<Receiver<Status>> = &state.see_goal;
                if let Some(face_status) = face_state {
                    match face_status.recv() {
                        Ok(status) => {
                            match status {
                                Status::Success => {
                                    // do nothing just say success
                                    (Status::Success, args.dt)
                                },
                                Status::Failure => {
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
                    face_task(tx, target);
                    state.see_goal = Some(rx);
                    let receiver = state.see_goal.as_ref().unwrap();
                    let status = receiver.recv().unwrap();
                    (status, args.dt)
                }
            },
            MissionLogic::Swim(target) => {
                // Alternative to face towards listed below
                // Add action to swim
                let swim_state: &Option<Receiver<Status>> = &state.see_goal;
                if let Some(swim_status) = swim_state {
                    match swim_status.recv() {
                        Ok(status) => {
                            match status {
                                Status::Success => {
                                    // do nothing just say success
                                    (Status::Success, args.dt)
                                },
                                Status::Failure => {
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
                    swim_task(tx, target);
                    state.see_goal = Some(rx);
                    let receiver = state.see_goal.as_ref().unwrap();
                    let status = receiver.recv().unwrap();
                    (status, args.dt)
                }
            }
        }
     );

    info!("Running mission");
    Ok(())
}

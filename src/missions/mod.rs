//! Dispatches named missions and handles cancellation during execution.

use crate::{
    logging::{info, Result},
    missions::tasks::{face_task, style_task, swim_task},
};
use bonsai_bt::{Event, Status, Timer, UpdateArgs, BT, RUNNING};
use std::collections::HashMap;
use std::sync::mpsc::{channel, Receiver, TryRecvError};
use tokio::{
    spawn,
    time::{sleep, Duration},
};
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
    SlalomWhite,
    SlalomRed,
    Shark,
    Arrow,
}

// Enum for actions by sea wolf in water
#[derive(Clone, Debug, PartialEq)]
pub enum MissionLogic {
    FaceTowards(Target),
    Swim(Target),
    Style,
}

// Advances the mission behavior tree by one tick.
// IDK how to use showdown token thing so for now just skip it
pub async fn run_mission(
    state: &mut SeaWolfState,
    timer: &mut Timer,
    bt: &mut BT<MissionLogic, HashMap<String, i32>>,
) -> Result<()> {
    info!("Running mission");
    // Bonsai advances through a behavior tree one tick at a time. Keep
    // ticking while an async mission task is running.
    loop {
        let dt = timer.get_dt();
        let event: Event = UpdateArgs { dt }.into();
        let result = bt.tick(&event, &mut |args: bonsai_bt::ActionArgs<
            Event,
            MissionLogic,
        >, _| {
            match args.action {
                MissionLogic::FaceTowards(target) => {
                    start_task_if_needed(state, |tx| face_task(tx, target.clone()));
                    poll_task(state, args.dt)
                }
                MissionLogic::Swim(target) => {
                    start_task_if_needed(state, |tx| swim_task(tx, target.clone(), 1.0));
                    poll_task(state, args.dt)
                }
                MissionLogic::Style => {
                    start_task_if_needed(state, style_task);
                    poll_task(state, args.dt)
                }
            }
        });

        match result {
            Some((Status::Running, _)) => sleep(Duration::from_millis(10)).await,
            Some((Status::Success, _)) => return Ok(()),
            Some((Status::Failure, _)) => {
                return Err(color_eyre::eyre::eyre!("Mission behavior tree failed"));
            }
            None => return Err(color_eyre::eyre::eyre!("Mission behavior tree stopped")),
        }
    }
}

fn start_task_if_needed<F, Fut>(state: &mut SeaWolfState, task: F)
where
    F: FnOnce(std::sync::mpsc::Sender<Status>) -> Fut,
    Fut: std::future::Future<Output = ()> + Send + 'static,
{
    if state.see_goal.is_none() {
        let (tx, rx) = channel();
        spawn(task(tx));
        state.see_goal = Some(rx);
    }
}

fn poll_task(state: &mut SeaWolfState, dt: f64) -> (Status, f64) {
    let receiver = state.see_goal.as_ref().expect("task receiver missing");
    match receiver.try_recv() {
        Ok(status) => {
            state.see_goal = None;
            (status, dt)
        }
        Err(TryRecvError::Empty) => RUNNING,
        Err(TryRecvError::Disconnected) => {
            state.see_goal = None;
            (Status::Failure, dt)
        }
    }
}

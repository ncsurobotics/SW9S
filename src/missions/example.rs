use tokio::{io::WriteHalf, select};
use tokio_serial::SerialStream;

use crate::{act_nest, missions::action_context::GetZedRos2};

use super::{
    action::{
        Action, ActionChain, ActionConcurrent, ActionConditional, ActionExec, ActionSequence,
        RaceAction,
    },
    action_context::{FrontCamIO, GetControlBoard},
    basic::DelayAction,
    comms::StartBno055,
    extra::{AlwaysTrue, OutputType, UnwrapAction},
    movement::{Descend, Stability2Movement, Stability2Pos},
};
use tokio_util::sync::CancellationToken;

/// Example function for Action system
///
/// Runs two nested actions in order: delaying and descending in
/// parallel, followed by a delay.
pub fn initial_descent<
    'a,
    Con: Send + Sync + GetControlBoard<WriteHalf<SerialStream>>,
    T: Send + Sync + 'a,
>(
    context: &'a Con,
) -> impl ActionExec<T> + 'a
where
    DelayAction: ActionExec<T>,
{
    ActionSequence::new(
        ActionConcurrent::new(DelayAction::new(1.0), Descend::new(context, -0.5)),
        DelayAction::new(1.0),
    )
}

pub fn pid_test<
    Con: Send + Sync + GetControlBoard<WriteHalf<SerialStream>> + FrontCamIO,
>(
    context: &Con,
) -> impl ActionExec<()> + '_ {
    let depth: f32 = -1.6;

    act_nest!(
        ActionSequence::new,
        ActionConcurrent::new(
            ActionChain::new(
                Stability2Movement::new(
                    context,
                    Stability2Pos::new(0.0, 0.0, 0.0, 0.0, None, depth),
                ),
                OutputType::<()>::default()
            ),
            StartBno055::new(context),
        ),
        act_nest!(
            ActionSequence::new,
            ActionChain::new(DelayAction::new(5.0), OutputType::<()>::default(),),
            ActionChain::new(
                Stability2Movement::new(
                    context,
                    Stability2Pos::new(0.0, 0.0, 0.0, 0.0, Some(45.0), depth),
                ),
                OutputType::<()>::default()
            ),
            DelayAction::new(10.0),
        ),
    )
}

/// Example function for Action system
///
/// Runs a conditional: delaying if true, descending otherwise.
pub fn always_wait<T: Send + Sync>(context: &T) -> impl Action + '_ {
    ActionConditional::new(
        AlwaysTrue::new(),
        DelayAction::new(1.0),
        Descend::new(context, -0.5),
    )
}

pub fn sequence_conditional<Con: Send + Sync + GetControlBoard<WriteHalf<SerialStream>>>(
    context: &Con,
) -> impl ActionExec<()> + '_ {
    ActionSequence::new(
        ActionSequence::new(DelayAction::new(1.0), Descend::new(context, -1.0)),
        ActionConditional::new(
            AlwaysTrue::new(),
            DelayAction::new(1.0),
            UnwrapAction::new(Descend::new(context, -0.5)),
        ),
    )
}

pub fn race_conditional<Con: Send + Sync + GetControlBoard<WriteHalf<SerialStream>>>(
    context: &Con,
) -> impl ActionExec<()> + '_ {
    ActionConditional::new(
        AlwaysTrue::new(),
        DelayAction::new(1.0),
        RaceAction::new(
            UnwrapAction::new(Descend::new(context, -0.5)),
            DelayAction::new(1.0),
        ),
    )
}

/// Function to demonstrate use of act_nest
pub fn race_many<Con: Send + Sync + GetControlBoard<WriteHalf<SerialStream>>>(
    _context: &Con,
) -> impl ActionExec<bool> + '_ {
    ActionSequence::<bool, _, _>::new(
        act_nest!(
            RaceAction::new,
            AlwaysTrue::new(),
            AlwaysTrue::new(),
            AlwaysTrue::new(),
            AlwaysTrue::new(),
            AlwaysTrue::new()
        ),
        AlwaysTrue::new(),
    )
}

pub async fn zed_test<
    Con: Send + Sync + GetControlBoard<WriteHalf<SerialStream>> + GetZedRos2,
>(
    context: &Con,
) {
    let zed = context.get_zed_ros2();

    loop {
        if let Some(pose) = zed.latest_pose().await {
            #[cfg(feature = "logging")]
            logln!(
                "Pose data received: x {} y {} z {}",
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            );
        } else {
            #[cfg(feature = "logging")]
            logln!("No pose data received");
        }
        let mut delay = DelayAction::new(0.5);
        delay.execute().await;
    }
}

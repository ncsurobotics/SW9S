pub mod auv_control_board;
pub mod control_board;
#[cfg(feature = "meb")]
pub mod meb;
#[cfg(feature = "ros")]
pub mod zed_ros2;

#[macro_export]
macro_rules! write_stream_mutexed {
    ( $stream_mutex:expr, $string:expr ) => {{
        $stream_mutex
            .lock()
            .await
            .write_all($string.as_bytes())
            .await
            .unwrap()
    }};
}

/// ROS2-compatible message type definitions for ZED camera topics.
///
/// These types mirror the standard ROS2 message definitions and are encoded
/// using CDR (Common Data Representation) serialization. They are used with
/// the zenoh-ros2dds bridge, which transparently forwards ROS2 DDS traffic
/// to Zenoh while preserving the CDR payload format.
///
/// Field order must exactly match the ROS2 IDL definitions — CDR is positional.
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// builtin_interfaces
// ---------------------------------------------------------------------------

/// builtin_interfaces/Time
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Time {
    pub sec: i32,
    pub nanosec: u32,
}

// ---------------------------------------------------------------------------
// std_msgs
// ---------------------------------------------------------------------------

/// std_msgs/Header
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Header {
    pub stamp: Time,
    pub frame_id: String,
}

// ---------------------------------------------------------------------------
// sensor_msgs
// ---------------------------------------------------------------------------

/// sensor_msgs/Image
///
/// Published on: `{namespace}/depth/depth_registered`
/// ZED encoding for depth: `"32FC1"` (32-bit float, single channel, metres)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Image {
    pub header: Header,
    /// Image height in pixels.
    pub height: u32,
    /// Image width in pixels.
    pub width: u32,
    /// Pixel encoding string (e.g. `"32FC1"`, `"bgr8"`).
    pub encoding: String,
    /// `0` = little-endian, `1` = big-endian (for multi-byte pixel types).
    pub is_bigendian: u8,
    /// Row stride in bytes (`width * bytes_per_pixel`).
    pub step: u32,
    /// Raw pixel data, length = `step * height`.
    pub data: Vec<u8>,
}

/// sensor_msgs/PointField — describes a single named field in a PointCloud2.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointField {
    pub name: String,
    /// Byte offset of this field from the start of a point.
    pub offset: u32,
    /// Data type (see `PointField::*` constants in ROS2).
    pub datatype: u8,
    /// Number of elements in the field (usually 1).
    pub count: u32,
}

/// sensor_msgs/PointCloud2
///
/// Published on: `{namespace}/point_cloud/cloud_registered`
/// ZED format: XYZRGBA organised cloud (`height * width` points per frame).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PointCloud2 {
    pub header: Header,
    /// Cloud height (number of rows; 1 for unordered clouds).
    pub height: u32,
    /// Cloud width (number of columns / points per row).
    pub width: u32,
    /// Description of each named field within a point (x, y, z, rgb, …).
    pub fields: Vec<PointField>,
    /// `true` if data is big-endian.
    pub is_bigendian: bool,
    /// Length in bytes of one point entry.
    pub point_step: u32,
    /// Length in bytes of one row.
    pub row_step: u32,
    /// Packed point data, length = `row_step * height`.
    pub data: Vec<u8>,
    /// `true` if there are no invalid (NaN/Inf) points.
    pub is_dense: bool,
}

// ---------------------------------------------------------------------------
// geometry_msgs
// ---------------------------------------------------------------------------

/// geometry_msgs/Point — 3D position in metres.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// geometry_msgs/Quaternion — unit quaternion orientation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// geometry_msgs/Pose — position + orientation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

/// geometry_msgs/PoseStamped — timestamped pose in a reference frame.
///
/// Published on: `{namespace}/pose`
/// ZED frame: `"map"` (world-locked odometry pose of the left camera).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

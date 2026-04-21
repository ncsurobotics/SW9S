use anyhow::Result;
use futures::StreamExt;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::Mutex;
use zenoh::Session;

use super::zed_messages::{Image, PointCloud2, PoseStamped};
use crate::config::ZedRos2Config;

pub mod zed_interfaces {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Keypoint2Di {
        pub kp: [u32; 2],
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Keypoint2Df {
        pub kp: [f32; 2],
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Keypoint3D {
        pub kp: [f32; 3],
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct BoundingBox2Di {
        pub corners: [Keypoint2Di; 4],
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct BoundingBox2Df {
        pub corners: [Keypoint2Df; 4],
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct BoundingBox3D {
        pub corners: [Keypoint3D; 8],
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Skeleton2D {
        pub keypoints: Vec<Keypoint2Df>,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Skeleton3D {
        pub keypoints: Vec<Keypoint3D>,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Object {
        pub label: String,
        pub label_id: i16,
        pub sublabel: String,
        pub confidence: f32,
        pub position: [f32; 3],
        pub position_covariance: [f32; 6],
        pub velocity: [f32; 3],
        pub tracking_available: bool,
        pub tracking_state: i8,
        pub action_state: i8,
        pub bounding_box_2d: BoundingBox2Di,
        pub bounding_box_3d: BoundingBox3D,
        pub dimensions_3d: [f32; 3],
        pub skeleton_available: bool,
        pub body_format: i8,
        pub head_bounding_box_2d: BoundingBox2Df,
        pub head_bounding_box_3d: BoundingBox3D,
        pub head_position: [f32; 3],
        pub skeleton_2d: Skeleton2D,
        pub skeleton_3d: Skeleton3D,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct ObjectsStamped {
        pub header: ros2_interfaces_jazzy_serde::std_msgs::msg::Header,
        pub objects: Vec<Object>,
    }

    impl ros2_client::Message for ObjectsStamped {}
}

use zed_interfaces::ObjectsStamped;

#[derive(Clone)]
pub struct ZedRos2 {
    image: Arc<Mutex<Option<Image>>>,
    objects: Arc<Mutex<Option<ObjectsStamped>>>,
    pose: Arc<Mutex<Option<PoseStamped>>>,
    /// Keeps the Zenoh session alive for the lifetime of this handle.
    _session: Arc<Session>,
}

impl ZedRos2 {
    pub async fn new(config: &ZedRos2Config) -> Result<Self> {
        let session = Arc::new(zenoh::open(zenoh::Config::default()).await?);

        let depth = Arc::new(Mutex::new(None::<Image>));
        let cloud = Arc::new(Mutex::new(None::<PointCloud2>));
        let pose = Arc::new(Mutex::new(None::<PoseStamped>));

        let depth_ke = zenoh_key(&config.namespace, &config.depth_topic);
        let cloud_ke = zenoh_key(&config.namespace, &config.cloud_topic);
        let pose_ke = zenoh_key(&config.namespace, &config.pose_topic);

        spawn_subscriber::<Image>(&session, depth_ke, depth.clone()).await?;
        spawn_subscriber::<PointCloud2>(&session, cloud_ke, cloud.clone()).await?;
        spawn_subscriber::<PoseStamped>(&session, pose_ke, pose.clone()).await?;

        Ok(Self {
            image,
            objects,
            pose,
            _session: session,
        })
    }

    pub async fn latest_image(&self) -> Option<Image> {
        self.image.lock().await.clone()
    }

    pub async fn latest_objects(&self) -> Option<ObjectsStamped> {
        self.objects.lock().await.clone()
    }

    pub async fn latest_pose(&self) -> Option<PoseStamped> {
        self.pose.lock().await.clone()
    }
}

/// Declares a Zenoh subscriber for `key_expr` and spawns a background task
/// that decodes each CDR sample into `T` and stores it in `cache`.
async fn spawn_subscriber<T>(
    session: &Session,
    key_expr: String,
    cache: Arc<Mutex<Option<T>>>,
) -> Result<()>
where
    T: serde::de::DeserializeOwned + Send + 'static,
{
    let sub = session.declare_subscriber(key_expr.as_str()).await?;
    tokio::spawn(async move {
        while let Ok(sample) = sub.recv_async().await {
            let raw = sample.payload().to_bytes();
            match decode_cdr::<T>(raw.as_ref()) {
                Ok(msg) => *cache.lock().await = Some(msg),
                Err(e) => eprintln!(
                    "[zed_ros2] CDR decode error on {}: {e}",
                    sample.key_expr()
                ),
            }
        }
    });
    Ok(())
}

/// Builds a Zenoh key expression from a ROS2-style namespace + topic.
///
/// Both the zenoh-ros2dds bridge and oxidros-zenoh follow the convention of
/// stripping the leading `/` and prefixing with `rt/` ("ROS2 topic"):
///   `/zed/zed_node/pose`              → `rt/zed/zed_node/pose`
///   namespace `/zed/zed_node` + `depth/depth_registered`
///                                      → `rt/zed/zed_node/depth/depth_registered`
fn zenoh_key(namespace: &str, topic: &str) -> String {
    let full = if topic.starts_with('/') {
        topic.to_string()
    } else {
        format!("{}/{}", namespace.trim_end_matches('/'), topic)
    };
    format!("rt/{}", full.trim_start_matches('/'))
}

/// Decodes a CDR-encoded ROS2 message.
///
/// ROS2 prepends a 4-byte RTPS encapsulation header before the CDR payload:
///   `[0x00, 0x01, 0x00, 0x00]`  (little-endian CDR — standard on ARM/x86)
/// The `cdr` crate handles the remaining bytes as standard CDR-LE.
fn decode_cdr<T: serde::de::DeserializeOwned>(bytes: &[u8]) -> Result<T> {
    if bytes.len() < 4 {
        bail!("CDR payload too short ({} bytes)", bytes.len());
    }
    cdr::deserialize::<T>(&bytes[4..]).map_err(|e| anyhow::anyhow!("CDR: {e}"))
}

fn rerun_image_from_ros_image(msg: &Image) -> Option<rerun::Image> {
    let (color_model, bytes_per_pixel) = match msg.encoding.as_str() {
        "rgb8" => (rerun::ColorModel::RGB, 3usize),
        "bgr8" => (rerun::ColorModel::BGR, 3usize),
        "rgba8" => (rerun::ColorModel::RGBA, 4usize),
        "bgra8" => (rerun::ColorModel::BGRA, 4usize),
        "mono8" | "8UC1" => (rerun::ColorModel::L, 1usize),
        _ => return None,
    };

    let width = msg.width as usize;
    let height = msg.height as usize;
    if width == 0 || height == 0 {
        return None;
    }

    let row_len = width.checked_mul(bytes_per_pixel)?;
    let step = msg.step as usize;
    if step < row_len {
        return None;
    }

    let expected_len = step.checked_mul(height)?;
    if msg.data.len() < expected_len {
        return None;
    }

    let data = if step == row_len {
        msg.data.clone()
    } else {
        let mut packed = Vec::with_capacity(row_len.checked_mul(height)?);
        for row in msg.data.chunks(step).take(height) {
            if row.len() < row_len {
                return None;
            }
            packed.extend_from_slice(&row[..row_len]);
        }
        packed
    };

    Some(rerun::Image::from_color_model_and_bytes(
        data,
        [msg.width, msg.height],
        color_model,
        rerun::ChannelDatatype::U8,
    ))
}

fn log_objects_to_rerun(rec: &rerun::RecordingStream, msg: &ObjectsStamped) {
    if msg.objects.is_empty() {
        let _ = rec.log("zed_ros2/image/objects", &rerun::Boxes2D::clear_fields());
        let _ = rec.log("zed_ros2/objects/log", &rerun::TextLog::new("0 detections"));
        return;
    }

    let mut mins = Vec::new();
    let mut sizes = Vec::new();
    let mut labels = Vec::new();

    for object in &msg.objects {
        let points = &object.bounding_box_2d.corners;

        let min_x = points
            .iter()
            .map(|point| point.kp[0] as f32)
            .fold(f32::INFINITY, f32::min);
        let min_y = points
            .iter()
            .map(|point| point.kp[1] as f32)
            .fold(f32::INFINITY, f32::min);
        let max_x = points
            .iter()
            .map(|point| point.kp[0] as f32)
            .fold(f32::NEG_INFINITY, f32::max);
        let max_y = points
            .iter()
            .map(|point| point.kp[1] as f32)
            .fold(f32::NEG_INFINITY, f32::max);

        let size_x = (max_x - min_x).max(0.0);
        let size_y = (max_y - min_y).max(0.0);

        if !min_x.is_finite()
            || !min_y.is_finite()
            || !size_x.is_finite()
            || !size_y.is_finite()
            || size_x == 0.0
            || size_y == 0.0
        {
            continue;
        }

        mins.push((min_x, min_y));
        sizes.push((size_x, size_y));
        labels.push(if object.label.is_empty() {
            format!("id {}", object.label_id)
        } else {
            format!("{} ({:.0}%)", object.label, object.confidence)
        });
    }

    if mins.is_empty() {
        let _ = rec.log("zed_ros2/image/objects", &rerun::Boxes2D::clear_fields());
        let _ = rec.log("zed_ros2/objects/log", &rerun::TextLog::new("0 detections"));
    } else {
        let detection_count = labels.len();
        let boxes = rerun::Boxes2D::from_mins_and_sizes(mins, sizes).with_labels(labels);
        let _ = rec.log("zed_ros2/image/objects", &boxes);
        let _ = rec.log(
            "zed_ros2/objects/log",
            &rerun::TextLog::new(format!("{detection_count} detections")),
        );
    }
}

fn log_pose_to_rerun(rec: &rerun::RecordingStream, msg: &PoseStamped) {
    let position = [
        msg.pose.position.x as f32,
        msg.pose.position.y as f32,
        msg.pose.position.z as f32,
    ];
    let quaternion = [
        msg.pose.orientation.x as f32,
        msg.pose.orientation.y as f32,
        msg.pose.orientation.z as f32,
        msg.pose.orientation.w as f32,
    ];

    let _ = rec.log(
        "zed_ros2/pose",
        &rerun::Transform3D::new()
            .with_translation(position)
            .with_quaternion(quaternion),
    );
    let _ = rec.log("zed_ros2/pose/position", &rerun::Points3D::new([position]));
}

use anyhow::{anyhow, Result};
use serde::{de::DeserializeOwned, Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::Mutex;
use zenoh::Session;

use crate::config::ZedRos2Config;

// Standard ROS2 message types
pub mod ros_interfaces {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Time {
        pub sec: i32,
        pub nanosec: u32,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Header {
        pub stamp: Time,
        pub frame_id: String,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Image {
        pub header: Header,
        pub height: u32,
        pub width: u32,
        pub encoding: String,
        pub is_bigendian: u8,
        pub step: u32,
        pub data: Vec<u8>,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Point {
        pub x: f64,
        pub y: f64,
        pub z: f64,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Quaternion {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct Pose {
        pub position: Point,
        pub orientation: Quaternion,
    }

    #[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
    pub struct PoseStamped {
        pub header: Header,
        pub pose: Pose,
    }
}

// ZED Interfaces Message Types
pub mod zed_interfaces {
    use super::*;
    pub use super::ros_interfaces::{Header, Time};

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
        pub header: Header,
        pub objects: Vec<Object>,
    }
}

use ros_interfaces::{Image, PoseStamped};
use zed_interfaces::ObjectsStamped;

// Zenoh-based ROS2 client
#[derive(Clone)]
pub struct ZedRos2 {
    image: Arc<Mutex<Option<Image>>>,
    objects: Arc<Mutex<Option<ObjectsStamped>>>,
    pose: Arc<Mutex<Option<PoseStamped>>>,
    _session: Session,
}

impl ZedRos2 {
    // Open a Zenoh session and spawn one subscriber per topic
    pub async fn new(config: &ZedRos2Config) -> Result<Self> {
        let session = open_session(config).await?;
        let domain_id = ros_domain_id();

        let image = spawn_topic_cache::<Image>(
            &session,
            topic_key(domain_id, &config.image_topic),
            |msg| {
                if let Some(img) = rerun_image_from_ros_image(msg) {
                    let _ = crate::get_recording().log("zed_ros2/image", &img);
                }
            },
        )
        .await?;

        let objects = spawn_topic_cache::<ObjectsStamped>(
            &session,
            topic_key(domain_id, &config.objects_topic),
            |msg| log_objects_to_rerun(&crate::get_recording(), msg),
        )
        .await?;

        let pose = spawn_topic_cache::<PoseStamped>(
            &session,
            topic_key(domain_id, &config.pose_topic),
            |msg| log_pose_to_rerun(&crate::get_recording(), msg),
        )
        .await?;

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

// Load the Zenoh config from file when given, else default to the local router
async fn open_session(config: &ZedRos2Config) -> Result<Session> {
    let zenoh_config = match &config.zenoh_config {
        Some(path) => zenoh::Config::from_file(path)
            .map_err(|e| anyhow!("failed to load zenoh config {path}: {e}"))?,
        None => {
            let mut default_config = zenoh::Config::default();
            default_config
                .insert_json5("mode", r#""client""#)
                .map_err(|e| anyhow!("failed to set zenoh mode: {e}"))?;
            default_config
                .insert_json5("connect/endpoints", r#"["tcp/localhost:7447"]"#)
                .map_err(|e| anyhow!("failed to set zenoh endpoints: {e}"))?;
            default_config
        }
    };
    zenoh::open(zenoh_config)
        .await
        .map_err(|e| anyhow!("failed to open zenoh session: {e}"))
}

// ROS_DOMAIN_ID selects the rmw_zenoh key namespace, defaulting to 0
fn ros_domain_id() -> u32 {
    std::env::var("ROS_DOMAIN_ID")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0)
}

// rmw_zenoh publishes on <domain>/<topic>/<type>/<hash>, so match the last two chunks with wildcards
fn topic_key(domain_id: u32, topic: &str) -> String {
    let topic = topic.trim_matches('/');
    format!("{domain_id}/{topic}/*/*")
}

// Subscribe to a key expression
async fn spawn_topic_cache<T>(
    session: &Session,
    key: String,
    on_msg: impl Fn(&T) + Send + 'static,
) -> Result<Arc<Mutex<Option<T>>>>
where
    T: DeserializeOwned + Send + 'static,
{
    let cache: Arc<Mutex<Option<T>>> = Arc::new(Mutex::new(None));
    let subscriber = session
        .declare_subscriber(key.as_str())
        .await
        .map_err(|e| anyhow!("failed to subscribe to {key}: {e}"))?;

    let task_cache = cache.clone();
    tokio::spawn(async move {
        while let Ok(sample) = subscriber.recv_async().await {
            if let Ok(msg) =
                cdr::deserialize_from::<_, T, _>(sample.payload().reader(), cdr::size::Infinite)
            {
                on_msg(&msg);
                *task_cache.lock().await = Some(msg);
            }
        }
    });
    Ok(cache)
}

// Convert a ROS image into a rerun image
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
    let data_slice = msg.data.as_slice();
    if data_slice.len() < expected_len {
        return None;
    }

    let data: Vec<u8> = if step == row_len {
        data_slice.to_vec()
    } else {
        let mut packed = Vec::with_capacity(row_len.checked_mul(height)?);
        for row in data_slice.chunks(step).take(height) {
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

// Log detections as labeled 2D boxes
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
            .map(|p| p.kp[0] as f32)
            .fold(f32::INFINITY, f32::min);
        let min_y = points
            .iter()
            .map(|p| p.kp[1] as f32)
            .fold(f32::INFINITY, f32::min);
        let max_x = points
            .iter()
            .map(|p| p.kp[0] as f32)
            .fold(f32::NEG_INFINITY, f32::max);
        let max_y = points
            .iter()
            .map(|p| p.kp[1] as f32)
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

// Log the camera pose as a rerun transform
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

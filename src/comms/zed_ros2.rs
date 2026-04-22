use anyhow::Result;
use oxidros::prelude::*;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::Mutex;
use ros2_interfaces_jazzy_serde::std_msgs;

use super::zed_messages::{Image, PoseStamped};

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
        pub header: std_msgs::msg::Header,
        pub objects: Vec<Object>,
    }
}

use zed_interfaces::ObjectsStamped;

macro_rules! impl_type_support {
    ($type:ty, $type_name:expr) => {
        impl TypeSupport for $type {
            fn to_bytes(&self) -> ros2_types::Result<Vec<u8>> {
                let payload = cdr::serialize::<_, _, cdr::CdrLe>(self, cdr::Infinite)
                    .map_err(|e| ros2_types::Error::CdrError(e.to_string()))?;
                let mut bytes = vec![0x00, 0x01, 0x00, 0x00];
                bytes.extend(payload);
                Ok(bytes)
            }

            fn from_bytes(bytes: &[u8]) -> ros2_types::Result<Self> {
                let payload = if bytes.len() >= 4 { &bytes[4..] } else { bytes };
                cdr::deserialize(payload)
                    .map_err(|e| ros2_types::Error::CdrError(e.to_string()))
            }

            fn type_name() -> &'static str {
                $type_name
            }
        }
    };
}

impl_type_support!(Image, "sensor_msgs::msg::dds_::Image_");
impl_type_support!(PoseStamped, "geometry_msgs::msg::dds_::PoseStamped_");
impl_type_support!(ObjectsStamped, "zed_msgs::msg::dds_::ObjectsStamped_");

#[derive(Clone)]
pub struct ZedRos2 {
    image: Arc<Mutex<Option<Image>>>,
    objects: Arc<Mutex<Option<ObjectsStamped>>>,
    pose: Arc<Mutex<Option<PoseStamped>>>,
}

impl ZedRos2 {
    pub async fn new() -> Result<Self> {
        let ctx = Context::new()?;
        let node = ctx.create_node("zed_ros2", None)?;

        let image = Arc::new(Mutex::new(None::<Image>));
        let objects = Arc::new(Mutex::new(None::<ObjectsStamped>));
        let pose = Arc::new(Mutex::new(None::<PoseStamped>));

        spawn_subscriber::<Image>(&node, "/zed/zed_node/rgb/image_rect_color", image.clone())?;
        spawn_subscriber::<ObjectsStamped>(&node, "/zed/zed_node/obj_det/objects", objects.clone())?;
        spawn_subscriber::<PoseStamped>(&node, "/zed/zed_node/pose", pose.clone())?;

        Ok(Self { image, objects, pose })
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

fn spawn_subscriber<T>(
    node: &Node,
    topic: &str,
    cache: Arc<Mutex<Option<T>>>,
) -> Result<()>
where
    T: TypeSupport + Clone + Send + 'static,
{
    let mut sub = node.create_subscriber::<T>(topic, None)?;
    let topic = topic.to_string();
    tokio::spawn(async move {
        let mut sub = sub;
        loop {
            match sub.recv().await {
                Ok(msg) => *cache.lock().await = Some((*msg).clone()),
                Err(e) => {
                    eprintln!("[zed_ros2] subscriber error on {topic}: {e}");
                    break;
                }
            }
        }
    });
    Ok(())
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

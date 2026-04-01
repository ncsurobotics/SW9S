use anyhow::Result;
use futures::StreamExt;
use ros2_client::{
    Context, MessageTypeName, Name, NodeName, NodeOptions, DEFAULT_SUBSCRIPTION_QOS,
};
use ros2_interfaces_jazzy_serde::{geometry_msgs::msg::PoseStamped, sensor_msgs::msg::Image};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::Mutex;

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
    _context: Context,
    _node: Arc<ros2_client::Node>,
}

impl ZedRos2 {
    pub fn new(config: &ZedRos2Config) -> Result<Self> {
        let context = Context::new()?;
        let mut node =
            context.new_node(NodeName::new("/", "sw8s_zed_client")?, NodeOptions::new())?;

        let image = Arc::new(Mutex::new(None));
        let objects = Arc::new(Mutex::new(None));
        let pose = Arc::new(Mutex::new(None));

        let image_topic = node.create_topic(
            &Name::new("/", "image").unwrap(),
            MessageTypeName::new("sensor_msgs", "Image"),
            &DEFAULT_SUBSCRIPTION_QOS,
        )?;
        let objects_name = Name::new("/", "obj_det/objects").unwrap();
        let objects_topic = node
            .create_topic(
                &objects_name,
                MessageTypeName::new("zed_interfaces", "ObjectsStamped"),
                &DEFAULT_SUBSCRIPTION_QOS,
            )
            .or_else(|_| {
                node.create_topic(
                    &objects_name,
                    MessageTypeName::new("zed_msgs", "ObjectsStamped"),
                    &DEFAULT_SUBSCRIPTION_QOS,
                )
            })?;
        let pose_topic = node.create_topic(
            &Name::new("/", "pose").unwrap(),
            MessageTypeName::new("geometry_msgs", "PoseStamped"),
            &DEFAULT_SUBSCRIPTION_QOS,
        )?;

        let image_sub = node.create_subscription::<Image>(&image_topic, None)?;
        let objects_sub = node.create_subscription::<ObjectsStamped>(&objects_topic, None)?;
        let pose_sub = node.create_subscription::<PoseStamped>(&pose_topic, None)?;

        let spinner = node.spinner()?;
        tokio::spawn(async move {
            let _ = spinner.spin().await;
        });

        let image_cache = image.clone();
        tokio::spawn(async move {
            let mut stream = Box::pin(image_sub.async_stream());
            while let Some(result) = stream.next().await {
                if let Ok((msg, _)) = result {
                    if let Some(img) = rerun_image_from_ros_image(&msg) {
                        let rec = crate::get_recording();
                        let _ = rec.log("zed_ros2/image", &img);
                    }

                    *image_cache.lock().await = Some(msg);
                }
            }
        });

        let objects_cache = objects.clone();
        tokio::spawn(async move {
            let mut stream = Box::pin(objects_sub.async_stream());
            while let Some(result) = stream.next().await {
                if let Ok((msg, _)) = result {
                    let rec = crate::get_recording();
                    log_objects_to_rerun(&rec, &msg);

                    *objects_cache.lock().await = Some(msg);
                }
            }
        });

        let pose_cache = pose.clone();
        tokio::spawn(async move {
            let mut stream = Box::pin(pose_sub.async_stream());
            while let Some(result) = stream.next().await {
                if let Ok((msg, _)) = result {
                    let rec = crate::get_recording();
                    log_pose_to_rerun(&rec, &msg);

                    *pose_cache.lock().await = Some(msg);
                }
            }
        });

        Ok(Self {
            image,
            objects,
            pose,
            _context: context,
            _node: Arc::new(node),
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

fn topic_name(namespace: &str, topic: &str) -> Result<Name> {
    println!("{namespace} ---- {topic}");
    if topic.starts_with('/') {
        let trimmed = topic.trim_start_matches('/');
        Name::new("/", trimmed).map_err(Into::into)
    } else {
        Name::new(namespace, topic).map_err(Into::into)
    }
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

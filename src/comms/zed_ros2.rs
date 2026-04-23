use anyhow::{Context, Result};
use std::sync::Arc;
use tokio::sync::Mutex;

use ros2_interfaces_jazzy_serde::geometry_msgs::msg::PoseStamped;
use ros2_interfaces_jazzy_serde::sensor_msgs::msg::{Image, Imu};

pub mod zed_interfaces {
    use ros2_interfaces_jazzy_serde::std_msgs::msg::Header;
    use serde::{Deserialize, Serialize};

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

const DOMAIN_ID: u32 = 0;

fn cdr_deserialize<T: for<'de> serde::Deserialize<'de>>(bytes: &[u8]) -> Result<T> {
    let payload = if bytes.len() >= 4 { &bytes[4..] } else { bytes };
    cdr::deserialize(payload).context("CDR deserialization failed")
}

fn read_payload(sample: &zenoh::sample::Sample) -> Vec<u8> {
    sample.payload().to_bytes().into_owned()
}

fn ros_key(domain_id: u32, topic: &str) -> String {
    let stripped = topic.trim_start_matches('/');
    format!("{domain_id}/{stripped}/**")
}

fn insert_json5(config: &mut zenoh::Config, key: &str, value: &str) -> Result<()> {
    config
        .insert_json5(key, value)
        .map_err(|e| anyhow::anyhow!("zenoh config key '{}': {}", key, e))
}

async fn build_session() -> Result<Arc<zenoh::Session>> {
    let mut config = zenoh::Config::default();

    insert_json5(&mut config, "connect/endpoints", r#"["tcp/localhost:7447"]"#)?;
    insert_json5(&mut config, "transport/shared_memory/enabled", "true")?;

    zenoh::open(config)
        .await
        .map(Arc::new)
        .map_err(|e| anyhow::anyhow!("zenoh open failed: {e}"))
}

#[derive(Clone)]
pub struct ZedRos2 {
    image: Arc<Mutex<Option<Image>>>,
    depth: Arc<Mutex<Option<Image>>>,
    imu: Arc<Mutex<Option<Imu>>>,
    pose: Arc<Mutex<Option<PoseStamped>>>,
}

impl ZedRos2 {
    pub async fn new() -> Result<Self> {
        let session = build_session().await?;

        let image = Arc::new(Mutex::new(None::<Image>));
        let depth = Arc::new(Mutex::new(None::<Image>));
        let imu = Arc::new(Mutex::new(None::<Imu>));
        let pose = Arc::new(Mutex::new(None::<PoseStamped>));

        spawn_subscriber::<Image>(
            &session,
            "/zed/zed_node/rgb/image_rect_color",
            image.clone(),
        )
        .await?;
        spawn_subscriber::<Image>(
            &session,
            "/zed/zed_node/depth/depth_registered",
            depth.clone(),
        )
        .await?;
        spawn_subscriber::<Imu>(&session, "/zed/zed_node/imu/data", imu.clone()).await?;
        spawn_subscriber::<PoseStamped>(&session, "/zed/zed_node/pose", pose.clone()).await?;

        Ok(Self { image, depth, imu, pose })
    }

    pub async fn latest_image(&self) -> Option<Image> {
        self.image.lock().await.clone()
    }

    pub async fn latest_depth(&self) -> Option<Image> {
        self.depth.lock().await.clone()
    }

    pub async fn latest_imu(&self) -> Option<Imu> {
        self.imu.lock().await.clone()
    }

    pub async fn latest_pose(&self) -> Option<PoseStamped> {
        self.pose.lock().await.clone()
    }
}

async fn spawn_subscriber<T>(
    session: &Arc<zenoh::Session>,
    topic: &str,
    cache: Arc<Mutex<Option<T>>>,
) -> Result<()>
where
    T: for<'de> serde::Deserialize<'de> + Clone + Send + 'static,
{
    let key = ros_key(DOMAIN_ID, topic);
    let topic_str = topic.to_string();

    let subscriber = session
        .declare_subscriber(&key)
        .await
        .map_err(|e| anyhow::anyhow!("failed to subscribe to {key}: {e}"))?;

    tokio::spawn(async move {
        let _sub = subscriber;
        loop {
            match _sub.recv_async().await {
                Ok(sample) => {
                    let bytes = read_payload(&sample);
                    match cdr_deserialize::<T>(&bytes) {
                        Ok(msg) => *cache.lock().await = Some(msg),
                        Err(e) => eprintln!("[zed_ros2] CDR decode error on {topic_str}: {e}"),
                    }
                }
                Err(e) => {
                    eprintln!("[zed_ros2] subscriber closed on {topic_str}: {e}");
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

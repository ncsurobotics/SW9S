use anyhow::Result;
use futures::StreamExt;
use ros2_client::{
    Context, MessageTypeName, Name, NodeName, NodeOptions, DEFAULT_SUBSCRIPTION_QOS,
};
use ros2_interfaces_jazzy_serde::{
    geometry_msgs::msg::PoseStamped,
    sensor_msgs::msg::{Image, PointCloud2},
};
use std::sync::Arc;
use tokio::sync::Mutex;

use crate::config::ZedRos2Config;

#[derive(Clone)]
pub struct ZedRos2 {
    depth: Arc<Mutex<Option<Image>>>,
    cloud: Arc<Mutex<Option<PointCloud2>>>,
    pose: Arc<Mutex<Option<PoseStamped>>>,
    _context: Context,
    _node: Arc<ros2_client::Node>,
}

impl ZedRos2 {
    pub fn new(config: &ZedRos2Config) -> Result<Self> {
        let context = Context::new()?;
        let mut node = context.new_node(
            NodeName::new(config.namespace.as_str(), "sw9s_zed_client")?,
            NodeOptions::new(),
        )?;

        let depth = Arc::new(Mutex::new(None));
        let cloud = Arc::new(Mutex::new(None));
        let pose = Arc::new(Mutex::new(None));

        let depth_topic = node.create_topic(
            &topic_name(config.namespace.as_str(), config.depth_topic.as_str())?,
            MessageTypeName::new("sensor_msgs", "Image"),
            &DEFAULT_SUBSCRIPTION_QOS,
        )?;
        let cloud_topic = node.create_topic(
            &topic_name(config.namespace.as_str(), config.cloud_topic.as_str())?,
            MessageTypeName::new("sensor_msgs", "PointCloud2"),
            &DEFAULT_SUBSCRIPTION_QOS,
        )?;
        let pose_topic = node.create_topic(
            &topic_name(config.namespace.as_str(), config.pose_topic.as_str())?,
            MessageTypeName::new("geometry_msgs", "PoseStamped"),
            &DEFAULT_SUBSCRIPTION_QOS,
        )?;

        let depth_sub = node.create_subscription::<Image>(&depth_topic, None)?;
        let cloud_sub = node.create_subscription::<PointCloud2>(&cloud_topic, None)?;
        let pose_sub = node.create_subscription::<PoseStamped>(&pose_topic, None)?;

        let spinner = node.spinner()?;
        tokio::spawn(async move {
            let _ = spinner.spin().await;
        });

        let depth_cache = depth.clone();
        tokio::spawn(async move {
            let mut stream = Box::pin(depth_sub.async_stream());
            while let Some(result) = stream.next().await {
                if let Ok((msg, _)) = result {
                    *depth_cache.lock().await = Some(msg);
                }
            }
        });

        let cloud_cache = cloud.clone();
        tokio::spawn(async move {
            let mut stream = Box::pin(cloud_sub.async_stream());
            while let Some(result) = stream.next().await {
                if let Ok((msg, _)) = result {
                    *cloud_cache.lock().await = Some(msg);
                }
            }
        });

        let pose_cache = pose.clone();
        tokio::spawn(async move {
            let mut stream = Box::pin(pose_sub.async_stream());
            while let Some(result) = stream.next().await {
                if let Ok((msg, _)) = result {
                    *pose_cache.lock().await = Some(msg);
                }
            }
        });

        Ok(Self {
            depth,
            cloud,
            pose,
            _context: context,
            _node: Arc::new(node),
        })
    }

    pub async fn latest_depth(&self) -> Option<Image> {
        self.depth.lock().await.clone()
    }

    pub async fn latest_cloud(&self) -> Option<PointCloud2> {
        self.cloud.lock().await.clone()
    }

    pub async fn latest_pose(&self) -> Option<PoseStamped> {
        self.pose.lock().await.clone()
    }
}

fn topic_name(namespace: &str, topic: &str) -> Result<Name> {
    if topic.starts_with('/') {
        let trimmed = topic.trim_start_matches('/');
        Name::new("/", trimmed).map_err(Into::into)
    } else {
        Name::new(namespace, topic).map_err(Into::into)
    }
}

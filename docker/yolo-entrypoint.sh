#!/bin/bash
set -e

if [ -f /opt/ros/jazzy/setup.bash ]; then
    echo "[yolo-entrypoint] Sourcing ROS Jazzy"
    source /opt/ros/jazzy/setup.bash
    source /root/ros2_ws/install/setup.bash
    source /root/ros2_ws/venv/bin/activate
elif [ -f /opt/ros/kilted/setup.bash ]; then
    echo "[yolo-entrypoint] Sourcing ROS Kilted"
    source /opt/ros/kilted/setup.bash
    source /root/ros2_ws/install/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    echo "[yolo-entrypoint] Sourcing ROS Humble"
    source /opt/ros/humble/setup.bash
    source /opt/ros/humble/install/setup.bash
    source /root/ros2_ws/install/setup.bash
    source /opt/venv/bin/activate
else
    echo "[yolo-entrypoint] ERROR: No supported ROS installation found!" >&2
    exit 1
fi

echo "[yolo-entrypoint] Camera topics detected — launching YOLO."

ros2 launch yolo_bringup yolo.launch.py \
    model:=/root/ROS/yolov8n-face.pt \
    device:=cuda:0 \
    enable:=True \
    target_frame:=camera_link \
    input_image_topic:=/camera/camera/color/image_raw \
    input_depth_topic:=/camera/camera/depth/image_rect_raw \
    input_depth_info_topic:=/camera/camera/depth/camera_info \
    use_3d:=True \
    use_debug:=True

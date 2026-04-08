#!/bin/bash
set -e

source /opt/ros/kilted/setup.bash
source /root/ros2_ws/install/setup.bash

ros2 launch yolo_bringup yolo.launch.py\
  model:=/root/ROS/yolov8n-face.pt \
  device:=cuda:0 \
  enable:=True \
  target_frame:=camera_link \
  input_image_topic:=/camera/camera/color/image_raw \
  input_depth_topic:=/camera/camera/depth/image_rect_raw \
  input_depth_info_topic:=/camera/camera/depth/camera_info \
  use_3d:=True \
  use_debug:=True

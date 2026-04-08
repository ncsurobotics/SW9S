#!/bin/bash
set -e

source /ws/install/setup.bash

echo "Launching Driver Module"
ros2 launch depthai_ros_driver_v3 driver.launch.py rs_compat:=true &

DEPTHAI_PID=$!

echo "Waiting for DepthAI topics..."
sleep 5

wait $DEPTHAI_PID

#!/bin/bash
set -e

source /root/ws/install/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py &

DEPTHAI_PID=$!
wait $DEPTHAI_PID

#!/bin/bash
set -e

source /ws/install/setup.bash

echo "Launching Driver Module"
if [[ "$SW9_ARCH" == "kilted-latest" ]]; then

if [[ "$SW9_ARCH" == "kilted-arm64-latest" ]]; then
    echo "Running on ARM architecture."
    ros2 launch depthai_ros_driver driver.launch.py rs_compat:=true &
elif [[ "$SW9_ARCH" == "kilted-latest" ]]; then
    echo "Running on x86 architecture."
    ros2 launch depthai_ros_driver_v3 driver.launch.py rs_compat:=true &
else
    echo "SW9_ARCH is unknown or not set. Current value: $SW9_ARCH"
fi

DEPTHAI_PID=$!
wait $DEPTHAI_PID

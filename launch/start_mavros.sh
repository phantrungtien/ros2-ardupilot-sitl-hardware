#!/bin/bash
# Script to start MAVROS connected to ArduPilot SITL

echo "========================================="
echo "Starting MAVROS for ArduPilot SITL"
echo "========================================="
echo ""
echo "Make sure SITL is running first!"
echo "Connection: udp://:14550@"
echo ""
echo "========================================="

# Source ROS2
source /opt/ros/humble/setup.bash

# Start MAVROS with direct command (more reliable than launch file)
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="udp://:14550@" \
    -p target_system_id:=1 \
    -p target_component_id:=1

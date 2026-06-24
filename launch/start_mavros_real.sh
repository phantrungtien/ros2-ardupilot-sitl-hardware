#!/bin/bash
# Script to start MAVROS for real hardware (Pixhawk 6C)
# Author: phantrungtien
# GitHub: https://github.com/phantrungtien

echo "========================================="
echo "Starting MAVROS for Real Hardware"
echo "========================================="
echo ""
echo "⚠️  SAFETY CHECKS:"
echo "  1. Propellers REMOVED?"
echo "  2. Drone secured on bench?"
echo "  3. Clear area around drone?"
echo "  4. RC transmitter ON?"
echo ""
read -p "All safety checks complete? (y/n): " -n 1 -r
echo ""

if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    echo "Safety checks not complete. Exiting..."
    exit 1
fi

echo ""
echo "Connection: TELEM3 GPIO UART to Pixhawk"
echo "Device: /dev/ttyAMA0:921600"
echo ""

# Ask for Mission Planner IP
read -p "Send telemetry to Mission Planner? (y/n): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]
then
    read -p "Enter Mission Planner laptop IP [10.73.2.236]: " LAPTOP_IP
    LAPTOP_IP=${LAPTOP_IP:-10.73.2.236}
    GCS_PARAM="-p gcs_url:=udp://@$LAPTOP_IP:14550"

    echo ""
    echo "✓ GCS URL: Sending telemetry to $LAPTOP_IP:14550"
    echo ""
    echo "Mission Planner Setup:"
    echo "  1. Open Mission Planner on laptop ($LAPTOP_IP)"
    echo "  2. Click 'Connect' dropdown → UDP"
    echo "  3. Port: 14550"
    echo "  4. Click 'Connect'"
    echo ""
else
    GCS_PARAM=""
    echo ""
    echo "Running without GCS forwarding"
    echo ""
fi

echo "========================================="

# Source ROS2
source /opt/ros/humble/setup.bash

# Start MAVROS with direct command (more reliable than launch file)
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="/dev/ttyAMA0:921600" \
    -p target_system_id:=1 \
    -p target_component_id:=1 \
    $GCS_PARAM

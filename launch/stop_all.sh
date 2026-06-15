#!/bin/bash
# Script to stop both MAVROS and SITL

echo "========================================="
echo "Stopping MAVROS and SITL"
echo "========================================="
echo ""

# Stop MAVROS first
echo "Stopping MAVROS..."
pkill -9 mavros_node
sleep 1

# Stop SITL
echo "Stopping SITL..."
pkill -9 -f "sim_vehicle.py"
pkill -9 -f "arducopter"
pkill -9 -f "MAVProxy"
pkill -9 ArduCopter
sleep 1

# Verify everything is stopped
echo ""
echo "Checking for remaining processes..."
MAVROS_RUNNING=$(pgrep -x "mavros_node" | wc -l)
SITL_RUNNING=$(pgrep -f "sim_vehicle\|arducopter\|MAVProxy\|ArduCopter" | wc -l)

if [ $MAVROS_RUNNING -eq 0 ] && [ $SITL_RUNNING -eq 0 ]; then
    echo "✓ All processes stopped successfully"
else
    echo "WARNING: Some processes may still be running:"
    if [ $MAVROS_RUNNING -gt 0 ]; then
        echo "  - MAVROS processes: $MAVROS_RUNNING"
    fi
    if [ $SITL_RUNNING -gt 0 ]; then
        echo "  - SITL processes: $SITL_RUNNING"
    fi
fi

echo ""
echo "========================================="

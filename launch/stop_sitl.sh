#!/bin/bash
# Script to stop ArduPilot SITL

echo "========================================="
echo "Stopping ArduPilot SITL"
echo "========================================="

# Kill sim_vehicle.py and related processes
pkill -9 -f "sim_vehicle.py"
pkill -9 -f "arducopter"
pkill -9 -f "MAVProxy"

# Also kill any remaining ArduCopter processes
pkill -9 ArduCopter

# Check if any are still running
if pgrep -f "sim_vehicle\|arducopter\|MAVProxy\|ArduCopter" > /dev/null
then
    echo "WARNING: Some SITL processes may still be running:"
    ps aux | grep -E "sim_vehicle|arducopter|MAVProxy|ArduCopter" | grep -v grep
else
    echo "✓ SITL stopped successfully"
fi

echo "========================================="

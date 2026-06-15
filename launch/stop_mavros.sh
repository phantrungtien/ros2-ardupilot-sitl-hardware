#!/bin/bash
# Script to stop MAVROS

echo "========================================="
echo "Stopping MAVROS"
echo "========================================="

# Kill all mavros_node processes
pkill -9 mavros_node

# Check if any are still running
if pgrep -x "mavros_node" > /dev/null
then
    echo "ERROR: Some MAVROS processes are still running"
    ps aux | grep mavros_node
else
    echo "✓ MAVROS stopped successfully"
fi

echo "========================================="

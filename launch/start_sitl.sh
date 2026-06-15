#!/bin/bash
# Script to start ArduPilot SITL for use with MAVROS

echo "========================================="
echo "Starting ArduPilot SITL - ArduCopter"
echo "========================================="
echo ""
echo "SITL will start on the following ports:"
echo "  - MAVLink output 1: 127.0.0.1:14550"
echo "  - MAVLink output 2: 127.0.0.1:14551"
echo ""
echo "MAVROS will connect to port 14550"
echo ""
echo "========================================="

cd ~/ardupilot/ArduCopter

# Start SITL with map and console
sim_vehicle.py -v ArduCopter --map --console

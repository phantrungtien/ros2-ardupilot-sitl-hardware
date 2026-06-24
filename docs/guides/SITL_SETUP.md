# SITL Simulation Setup Guide

Complete guide to test your missions safely in ArduPilot SITL simulation.

---

## What is SITL?

**Software In The Loop (SITL)** allows you to test drone missions on your computer without any hardware. Perfect for:
- Developing and testing missions
- Learning autonomous flight
- Safe experimentation

---

## Quick Start

### Terminal 1: Start SITL

```bash
cd ~/ros2-ardupilot-sitl-hardware
./launch/start_sitl.sh
```

**What this does:**
- Starts ArduPilot ArduCopter simulation
- Opens map window and console
- Broadcasts MAVLink on ports 14550/14551

**Expected output:**
```
Starting ArduPilot SITL - ArduCopter
...
STABILIZE>
```

---

### Terminal 2: Start MAVROS

```bash
cd ~/ros2-ardupilot-sitl-hardware  
./launch/start_mavros.sh
```

**What this does:**
- Connects ROS2 MAVROS to SITL
- Publishes drone state as ROS2 topics

**Expected output:**
```
[INFO] [mavros]: FCU URL: udp://:14550@
[INFO] [mavros]: CON: Got HEARTBEAT, connected
```

---

### Terminal 3: Verify Connection

```bash
source ~/ros2-ardupilot-sitl-hardware/install/setup.bash

# Check connection state
ros2 topic echo /mavros/state --once
```

**Expected output:**
```yaml
connected: true
armed: false
mode: "STABILIZE"
```

✅ **Connection successful!**

---

## Run Your First Mission

```bash
cd ~/ros2-ardupilot-sitl-hardware
source install/setup.bash
python3 scripts/missions/mission_simple.py
```

**What happens:**
1. Sets GUIDED mode
2. Arms throttle
3. Takes off to 10 meters
4. Flies forward 10 meters
5. Returns to launch

**Watch the drone in the SITL map window!**

---

## Manual Control via ROS2

### Arm the Vehicle
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### Change to GUIDED Mode
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

### Takeoff to 10m
```bash
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10.0}"
```

### Land
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'LAND'}"
```

---

## Available ROS2 Topics

### Monitor Drone State
```bash
# Connection and mode
ros2 topic echo /mavros/state

# GPS position
ros2 topic echo /mavros/global_position/global

# Local position (meters from home)
ros2 topic echo /mavros/local_position/pose

# Battery voltage
ros2 topic echo /mavros/battery

# IMU data
ros2 topic echo /mavros/imu/data
```

### List All Topics
```bash
ros2 topic list | grep mavros
```

---

## Connection Details

| Parameter | Value |
|-----------|-------|
| **SITL MAVLink Output** | `127.0.0.1:14550` |
| **MAVROS Connection** | `udp://:14550@` |
| **Protocol** | MAVLink v2.0 |
| **Target System ID** | 1 |
| **Target Component ID** | 1 |

---

## Troubleshooting

### SITL won't start

**Problem:** `sim_vehicle.py: command not found`

**Solution:**
```bash
# Check PATH includes ArduPilot tools
echo $PATH | grep ardupilot

# If not, add to ~/.bashrc:
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
source ~/.bashrc
```

---

### MAVROS not connecting

**Problem:** Timeout or no heartbeat

**Solutions:**
1. Make sure SITL is running first
2. Check SITL shows MAVLink on port 14550:
   ```
   Link 1: Opened
   ```
3. Restart both SITL and MAVROS

---

### No topics appearing

**Problem:** `ros2 topic list` shows nothing

**Solution:**
```bash
# Source the workspace
source /opt/ros/humble/setup.bash
source ~/ros2-ardupilot-sitl-hardware/install/setup.bash

# Verify MAVROS is running
ros2 node list
# Should show: /mavros
```

---

### Build errors

**Problem:** Package not found after modifications

**Solution:**
```bash
cd ~/ros2-ardupilot-sitl-hardware
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## Stopping Everything

### Stop individual components:
```bash
./launch/stop_mavros.sh  # Stop MAVROS only
./launch/stop_sitl.sh    # Stop SITL only
```

### Stop everything at once:
```bash
./launch/stop_all.sh
```

---

## Next Steps

✅ SITL working  
✅ Mission tested in simulation  
→ Ready for [Real Hardware Setup](REAL_HARDWARE_SETUP.md)

---

**Author:** phantrungtien  
**GitHub:** https://github.com/phantrungtien

# MAVROS Mission Scripts

Python scripts for autonomous drone control via MAVROS and ArduPilot SITL.

---

## Available Scripts

### `mission_simple.py` - Complete Autonomous Mission ⭐

**Full autonomous mission sequence:**

1. ✅ Set **GUIDED** mode
2. ✅ **Arm** throttle
3. ✅ **Takeoff** to **10 meters**
4. ✅ **Move forward 10 meters**
5. ✅ **Wait 3 seconds**
6. ✅ **Return to Launch (RTL)**

**How to run:**
```bash
cd ~/ros2-ardupilot-sitl-hardware
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/missions/mission_simple.py
```

**Features:**
- ✅ Correct QoS settings for MAVROS compatibility
- ✅ Real-time altitude monitoring
- ✅ Position tracking with distance calculation
- ✅ Step-by-step execution with proper waits
- ✅ Clean, simple code without continuous timers
- ✅ Safe RTL at mission end

---

## Prerequisites

Before running any script, you need:

### 1. Start SITL (Terminal 1)
```bash
cd ~/ros2-ardupilot-sitl-hardware/launch
./start_sitl.sh
```

### 2. Start MAVROS (Terminal 2)
```bash
cd ~/ros2-ardupilot-sitl-hardware/launch
./start_mavros.sh
```

### 3. Verify Connection (Terminal 3)
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mavros/state --once
```
Make sure `connected: true`

---

## Script Parameters

You can modify parameters in `mission_simple.py`:

**Takeoff altitude:**
```python
self.takeoff(10.0)  # Change to desired altitude in meters
```

**Forward distance:**
```python
self.move_to_position(10.0, 0.0, 10.0, duration=15)
# First parameter = forward distance (X-axis)
# Second parameter = lateral distance (Y-axis)
# Third parameter = altitude to maintain
# duration = time to publish setpoints (seconds)
```

**Wait time:**
```python
for i in range(3, 0, -1):  # Change 3 to desired wait time
```

**Altitude tolerance:**
```python
self.wait_for_altitude(10.0, tolerance=0.5)  # ±0.5m
```

---

## Key Technical Details

### QoS Settings
MAVROS uses **BEST_EFFORT** reliability for pose topics. The script correctly configures:

```python
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

### Setpoint Timing
- **Does NOT** publish setpoints during takeoff (prevents interference)
- Only publishes setpoints **after** takeoff completes
- Publishes at 20Hz when moving to waypoints

### Frame of Reference
- Uses `map` frame for local position control
- X-axis = Forward (North)
- Y-axis = Right (East)
- Z-axis = Up (altitude)

---

## Safety Notes

- ⚠️ **Always test in SITL first** before using on real hardware
- ⚠️ Monitor the drone position in SITL map console
- ⚠️ Press `Ctrl+C` to interrupt mission at any time
- ⚠️ RTL mode automatically returns drone to launch point
- ⚠️ Ensure GPS lock in SITL before arming

---

## Troubleshooting

### Script can't connect to MAVROS
**Problem:** Services not found
**Solution:**
```bash
# Check MAVROS is running
ros2 node list | grep mavros

# Restart MAVROS if needed
cd ~/ros2-ardupilot-sitl-hardware/launch
./stop_mavros.sh
./start_mavros.sh
```

### Takeoff command accepted but drone doesn't climb
**Problem:** Position setpoints interfering
**Solution:** This is already fixed in `mission_simple.py` - setpoints only publish after takeoff

### Altitude not detected (stuck waiting for altitude)
**Problem:** QoS mismatch between script and MAVROS
**Solution:** Already fixed with BEST_EFFORT QoS profile

### Position control not working
**Problem:** Not in GUIDED mode
**Solution:** Script automatically sets GUIDED mode before commanding positions

---

## Creating Your Own Mission Scripts

Template structure:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped

class MyMission(Node):
    def __init__(self):
        super().__init__('my_mission')

        # QoS for MAVROS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Setup subscribers with correct QoS
        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_callback, qos_profile
        )

        # Setup service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

    def run_mission(self):
        # Your mission logic here
        pass

def main():
    rclpy.init()
    node = MyMission()
    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Useful ROS2 Commands

### Monitor Topics
```bash
# Check connection state
ros2 topic echo /mavros/state

# Monitor altitude
ros2 topic echo /mavros/local_position/pose

# Check GPS
ros2 topic echo /mavros/global_position/global
```

### Manual Control via Services
```bash
# Set GUIDED mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# Arm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Takeoff to 10m
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 10.0}"

# RTL
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'RTL'}"
```

---

---

Happy Flying! 🚁

# MAVROS Mission Scripts

Autonomous drone control via MAVROS and ArduPilot SITL.

> **Where the missions live:** the canonical, maintained missions are ROS2
> nodes in the **`simtofly_mavros_sitl`** package, run with `ros2 run` or via
> the all-in-one [`sitl_mission.launch.py`](../../src/simtofly_mavros_sitl/launch/sitl_mission.launch.py).
> This `scripts/missions/` folder keeps **standalone copies** of
> `mission_simple.py` and `trackinghuman.py` that you can run directly with
> `python3` (no `colcon build` needed) for quick experiments.

| Mission | Package executable | Standalone script | What it does |
|---------|--------------------|-------------------|--------------|
| `mission_simple` | ✅ | ✅ (here) | Takeoff → forward 10 m → RTL |
| `mission_circle` | ✅ | — | Velocity-based circular flight |
| `trackinghuman` | ✅ | ✅ (here) | Vision-driven human following |
| `verify_frame` | ✅ | — | ENU/yaw frame-convention probe |

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

**How to run (package executable — recommended):**
```bash
cd ~/ros2-ardupilot-sitl-hardware
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run simtofly_mavros_sitl mission_simple
```

**Or run the standalone copy directly (no build needed):**
```bash
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

### `mission_circle.py` - Circular Trajectory (velocity control) ⭕

Flies **N circles using velocity setpoints** (`/mavros/setpoint_raw/local`),
not position waypoints. Geometric controller: constant-speed tangent
feedforward at the drone's actual angular position + a radial P-term to hold
the circle; laps counted from the real swept angle; nose kept tangent.

**How to run (all-in-one launch):**
```bash
colcon build --packages-select simtofly_mavros_sitl
source install/setup.bash
ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_circle
```

> Note: `mission_circle` lives in the **`simtofly_mavros_sitl` package**
> (run via `ros2 run` / launch, not `python3 scripts/...`). The pure
> controller geometry is in `simtofly_mavros_sitl/circle_control.py` and is
> unit-tested in `test/test_circle_control.py`.

Tunables (`radius`, `speed`, `direction`, `laps`, `kp_r`, ...) sit at the top
of `mission_circle.py`. See the main
[README](../../README.md#fly-a-circle-velocity-control) for the full parameter
table and the bank-angle limit `φ ≈ atan(V²/Rg)`.

---

### `trackinghuman.py` - Vision-Driven Human Follow 🧍

Follows a person under MAVROS/GUIDED. It does **not** detect anything itself —
a separate vision node feeds it two topics, and it turns them into body-frame
velocity + yaw_rate setpoints on `/mavros/setpoint_raw/local`:

| Topic | Type | Meaning |
|-------|------|---------|
| `/follow/command` | `std_msgs/String` | `FOLLOW` \| `BACK_OFF` \| `SEARCH` \| `IDLE` |
| `/follow/target`  | `geometry_msgs/Vector3` | `x`=ang_x(deg), `y`=ang_y(deg), `z`=distance(m) |

Control is **yaw-and-go**: yaw to centre the person, then hold the `STANDOFF_M`
distance; altitude is held constant. Safety: streams at 20 Hz, hovers on vision
loss (`VISION_TIMEOUT`), RTLs after `LOST_RTL_SEC`, releases control if the
pilot leaves GUIDED, and caps altitude at `MAX_ALT`. Tunables sit at the top of
`trackinghuman.py`.

```bash
ros2 run simtofly_mavros_sitl trackinghuman
# or, via the all-in-one launch:
ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=trackinghuman
```

---

### `verify_frame.py` - Frame-Convention Probe 🧭

A one-shot diagnostic, **not** a mission. It flies GUIDED → arm → takeoff, then
commands `vx+`, `vy+` and `yaw=0` on `/mavros/setpoint_raw/local` and measures
the resulting ENU pose deltas, so you can confirm empirically how setpoint
commands map onto the pose frame before trusting velocity-based missions. It
prints a summary and RTLs.

```bash
ros2 run simtofly_mavros_sitl verify_frame
```

SITL-confirmed result (already baked into the missions): `vx+` → **+East**,
`vy+` → **+North**, `yaw=0` → nose points **East** (ENU).

---

## Prerequisites

> **Shortcut:** the all-in-one launch starts SITL **and** MAVROS **and** the
> mission for you (with Gazebo), so you can skip the manual steps below:
> `ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_simple`

To run a mission **manually** (headless SITL, no Gazebo) you need:

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
# First parameter  = X / East   (m)   <- ENU, not "forward"
# Second parameter = Y / North  (m)
# Third parameter  = Z / altitude to hold (m)
# duration         = how long to publish setpoints (seconds)
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

### Frame of Reference (ENU)
The MAVROS local frame is **ENU**, confirmed in SITL with `verify_frame`:
- **X-axis = East**
- **Y-axis = North**
- **Z-axis = Up** (altitude)
- **Yaw: 0 = East, +90° = North** (CCW positive)

So a velocity `vx>0` moves the drone **East** and `vy>0` moves it **North**
(not "forward"). `move_to_position(10, 0, 10)` therefore moves **+10 m East**
while holding 10 m altitude.

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
pkill -9 -x mavros_node
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

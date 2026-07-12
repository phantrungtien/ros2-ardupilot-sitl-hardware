# ROS2-ArduPilot SITL & Hardware Integration

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Copter%204.x-green)](https://ardupilot.org/)

 > Open-source framework to control ArduPilot drones with ROS2. Test missions in SITL simulation, then deploy to real hardware.



Validated on real flights with a Pixhawk 6C flight controller (TELEM3 serial link) and a companion computer (Raspberry Pi 4 or NVIDIA Jetson).

---

## Getting Started

| Step | Description | Guide |
|:----:|-------------|-------|
| 1 | **Install** - ROS2, MAVROS, ArduPilot SITL | [Installation Guide](docs/installation/installation-README.md) |
| 2 | **Use** - Test in simulation, deploy to hardware | [Workflow Guide](docs/WORKFLOW.md) |

> Already have ROS2 + MAVROS + ArduPilot installed? Skip to [Quick Start](#quick-start) below.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Your Mission Script                          │
│                      (Python / ROS2 Node)                           │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        MAVROS (ROS2)                                │
│              Translates ROS2 ↔ MAVLink Protocol                     │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    ▼                           ▼
        ┌───────────────────┐       ┌───────────────────┐
        │   SITL Simulator  │       │   Real Hardware   │
        │   (UDP:14550)     │       │  (/dev/ttyAMA0)   │
        │                   │       │   921600 baud     │
        │   ArduPilot       │       │  Pixhawk 6C       │
        │   Virtual Drone   │       │  Flight Controller│
        └───────────────────┘       └───────────────────┘
                                            │
                                            ▼
                                    ┌───────────────┐
                                    │    Drone      │
                                    └───────────────┘
```

**Same mission code works for both paths** - only the MAVROS connection changes.

---

## Key Features

- **Companion Computer Control** - Command drone autonomously from a companion computer in GUIDED mode
- **SITL & Hardware Testing** - Test missions in simulation, then deploy same code to real drone
- **Flight Validated** - Tested in autonomous GUIDED flight on real hardware (Pixhawk 6C)
- **Safety First** - Comprehensive pre-flight checks and emergency procedures
- **Ground Station Optional** - Mission Planner/QGroundControl for monitoring
- **Complete Workflow** - SITL testing → Bench testing → Flight testing documented
- **Open Hardware Reference** - Test platform fully documented (CAD, assembly, wiring)

---

## Quick Start

### Prerequisites
- **ROS2 Humble** on Ubuntu 22.04
- **ArduPilot SITL** (for simulation)
- **MAVROS** installed

**Don't have these?** See [Installation Guide](docs/installation/installation-README.md)

---

### Test in Simulation

```bash
# Clone and build
git clone https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware.git
cd ros2-ardupilot-sitl-hardware
colcon build
source install/setup.bash
```

**Option A — All-in-one launch (Gazebo + SITL + MAVROS + mission).**
One command opens Gazebo, ArduPilot SITL, MAVROS and the mission, each in its
own terminal window and ordered by *readiness* (each window waits for its
dependency, not a timer):

```bash
ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_simple
```

Swap the mission for `mission_circle`, `mission_gates`, `trackinghuman` or
`verify_frame`. The world follows the mission: `mission_gates` opens the gate course,
everything else opens the runway.

Requires Gazebo, the `ardupilot_gazebo` plugin, and the worlds/models repo at
`~/gz_ws` — see [Fly Through a Gate](#fly-through-a-gate) below for that one-time
setup. (No Gazebo? [SITL Setup](docs/guides/SITL_SETUP.md) covers the headless path
— Option B — which needs none of it.)

**Option B — Manual, headless SITL (no Gazebo), three terminals:**

```bash
chmod +x launch/*.sh          # first time only

# Terminal 1 — ArduPilot SITL
./launch/start_sitl.sh

# Terminal 2 — MAVROS
./launch/start_mavros.sh

# Terminal 3 — run a mission from the package
source install/setup.bash
ros2 run simtofly_mavros_sitl mission_simple

# When done, stop everything cleanly
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
```

**Watch your drone take off in the SITL map!**

---

### Fly a Circle (velocity control)

`mission_circle` flies N circles by streaming **velocity + tangent yaw**
setpoints (not position waypoints). It uses a geometric controller: a
constant-speed tangent feedforward computed at the drone's *actual* angular
position, plus a radial P-term that holds it on the circle. Laps are counted
from the real swept angle, not a timer.

```bash
# All-in-one: Gazebo + SITL + MAVROS + mission, each in its own window,
# ordered by readiness (not timers).
colcon build --packages-select simtofly_mavros_sitl
source install/setup.bash
ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_circle
```

Tune the run at the top of
`src/simtofly_mavros_sitl/simtofly_mavros_sitl/mission_circle.py`:

| Param | Default | Meaning |
|-------|--------:|---------|
| `radius` | 5.0 m | circle radius R |
| `speed` | 2.0 m/s | tangential speed V (→ ω = V/R) |
| `direction` | +1 | +1 = CCW, −1 = CW |
| `altitude` | 10.0 m | altitude held during the circle |
| `laps` | 2 | full circles before RTL |
| `kp_r` | 0.8 | radial stiffness — raise to tighten the radius |

**Physical limit:** centripetal accel `a = V²/R`, bank angle `φ ≈ atan(a/g)`.
Defaults give `a = 0.8 m/s²`, `φ ≈ 4.7°` (gentle). Raising `V` or lowering `R`
grows the bank angle fast — keep it flyable.

> **SITL-verified:** clean circle, correct direction, altitude held, nose
> tangent, stops at exactly N laps then RTL. The *real* radius settles ~10–12%
> wider than R at the default `kp_r` (velocity-loop lag an ideal-tracking model
> doesn't show); raise `kp_r` if you need it tighter.

The pure controller geometry lives in `circle_control.py` and is unit-tested
(`test/test_circle_control.py`), independent of ROS.

---

### Fly Through a Gate

`mission_gates` flies the drone through a racing ring using **only the onboard
camera** — colour to find the ring, **depth to range it**. It is never told where the
gate is: it finds it, ranges it, and flies through it.

Two halves, split across two packages:

| Half | Node | Does |
|------|------|------|
| **Vision** | `simtofly_vision/gate_detect` | Finds the orange ring (HSV → `cv2.fitEllipse`), **ranges it with the D435i depth stream**, publishes `/gate/obs` |
| **Control** | `simtofly_mavros_sitl/mission_gates` | Reconstructs where the ring is *in metres*, flies through it |

#### 1. Get the Gazebo world and models

The drone, the gate and the world live in a **separate repo** — an ArduPilot Iris
carrying a nose-mounted **Intel RealSense D435i** (colour + depth + stereo IR + IMU),
a racing ring, and the course world:

**→ [phantrungtien/gazebo_ardupilot_depth_camera](https://github.com/phantrungtien/gazebo_ardupilot_depth_camera)**

```bash
# The launch files expect it at ~/gz_ws — clone it there.
git clone https://github.com/phantrungtien/gazebo_ardupilot_depth_camera.git ~/gz_ws
cd ~/gz_ws

# ArduPilot's plugin is kept as a pristine clone (gitignored, never modified)
git clone https://github.com/ArduPilot/ardupilot_gazebo src/ardupilot_gazebo
cd src/ardupilot_gazebo && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
```

`~/gz_ws/setup.bash` is the single place that puts **both** repos on
`GZ_SIM_RESOURCE_PATH`; the launch files here source it, so you never set those
paths by hand. See that repo's README for the camera's intrinsics and for what the
simulator does *not* give you (depth noise, min-Z blind zone, frame conventions).

#### 2. Fly it

```bash
colcon build --packages-select simtofly_vision simtofly_mavros_sitl
source install/setup.bash

# 6 windows, and gate_detect opens its GUI so you can watch the ellipse it fits
ros2 launch simtofly_mavros_sitl gates_mission.launch.py

# or through the shared launch — same thing, no vision GUI
ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_gates
```

Both bring up the `ros_gz_bridge` that carries the depth image into ROS. Colour
needs no bridge — it still arrives as H.264/RTP on UDP 5600.

#### 3. What makes it hard: a 50 mm margin

The ring's clear opening is **0.371 m** and the drone is **0.321 m** across the prop
tips, and the ring **has collision** — so there are **50 mm** to spend, and spending
them is a crash. Three biases each exceed that budget on their own:

| Bias | Size | vs. the 50 mm |
|------|-----:|--------------:|
| A tolerance of 0.15 in *normalised pixels*, at commit range | ~154 mm | **3×** |
| The colour camera sits left of the drone's centreline | 32.5 mm | **65%** |
| 5° of nose-down pitch (the camera is welded to the airframe) | 123 mm | **2.5×** |

So the controller never servos pixels to zero. It reconstructs the ring's position in
**metres in the body frame** (where the camera offset lives), then rotates it into ENU
with the drone's **attitude quaternion** (where the pitch bias lives). Both biases
cancel by construction, and every threshold in the file is a real distance you can
compare against the 50 mm.

**The ring also goes blind before you reach it.** The detector fits an ellipse to the
ring's outer contour, so once the ring outgrows the frame that contour is cut and the
ring stops existing — at full size, dead ahead. The colour camera's *vertical*
half-FOV binds, so this happens below `0.491 / tan(21.25°) = 1.26 m`. The mission
therefore commits **while the ring is still whole**, and flies the last stretch on a
frozen world point off `/mavros/local_position` rather than on pixels.

Both halves have an offline self-check that needs no simulator:

```bash
python3 src/simtofly_vision/simtofly_vision/gate_detect.py --selfcheck
python3 src/simtofly_mavros_sitl/simtofly_mavros_sitl/mission_gates.py --selfcheck
```

> **SITL-verified:** the drone finds the ring, closes on it, commits, flies clean
> through the hole without touching it, then RTLs.

---

### Deploy to Real Hardware

**After SITL testing succeeds:**
```bash
# 1. Stop SITL completely
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'

# 2. Wire the Pixhawk 6C to the companion computer:
#    TELEM3 (SERIAL4) UART -> companion GPIO UART (/dev/ttyAMA0 @ 921600)

# 3. Start MAVROS for hardware (Terminal 1)
./launch/start_mavros_real.sh
# Answer the safety prompts

# 4. Run the SAME mission (Terminal 2)
source install/setup.bash
ros2 run simtofly_mavros_sitl mission_simple
```

**⚠️ CRITICAL:** Remove propellers for bench testing!

---

### Complete Workflow Guide

See **[WORKFLOW.md](docs/WORKFLOW.md)** for:
- Detailed SITL testing procedure
- Hardware transition steps
- Helper script reference
- Troubleshooting common issues
- Best practices

---

## Repository Structure

```
├── src/                                  # ROS2 packages
│   ├── simtofly_mavros_sitl/             # SITL package: missions + launch + tests
│   │   ├── simtofly_mavros_sitl/
│   │   │   ├── guided_base.py            # shared GUIDED -> arm -> takeoff helpers
│   │   │   ├── circle_control.py         # pure circle geometry (no ROS, unit-tested)
│   │   │   ├── mission_simple.py         # takeoff -> forward -> RTL
│   │   │   ├── mission_circle.py         # velocity-based circular flight
│   │   │   ├── mission_gates.py          # fly through a ring, by camera alone
│   │   │   ├── trackinghuman.py          # vision-driven human follow
│   │   │   └── verify_frame.py           # ENU/yaw frame-convention probe
│   │   ├── launch/sitl_mission.launch.py # Gazebo + SITL + MAVROS + mission (all-in-one)
│   │   ├── launch/gates_mission.launch.py# same, gates-only, with the vision GUI on
│   │   └── test/test_circle_control.py   # unit tests for circle geometry
│   ├── simtofly_vision/                  # Vision nodes (publish what the missions fly by)
│   │   └── simtofly_vision/
│   │       ├── gate_detect.py            # ring detect + D435i depth ranging -> /gate/obs
│   │       └── person_follow_reid.py     # YOLO + OSNet ReID -> /follow/*
│   └── simtofly_mavros_real/             # Real-hardware MAVROS launch (Pixhawk 6C)
├── scripts/missions/                     # Standalone copies (run with python3)
├── launch/                               # Helper shell scripts (start SITL & MAVROS)
├── docs/                                 # Installation, workflow, hardware docs
└── README.md                             # You are here
```

The **Gazebo worlds and models are not in this repo** — the D435i-carrying Iris, the
gate ring and the course world live in
[gazebo_ardupilot_depth_camera](https://github.com/phantrungtien/gazebo_ardupilot_depth_camera),
cloned to `~/gz_ws`:

```
~/gz_ws/
├── setup.bash                            # puts BOTH repos on GZ_SIM_RESOURCE_PATH
├── src/drone_vision/                     # the models and world (this is the repo)
│   ├── models/realsense_d435i/           # colour + depth + stereo IR + IMU
│   ├── models/iris_with_d435i/           # iris + the camera, nose-mounted
│   ├── models/gate_ring/                 # the racing ring
│   └── worlds/gates_course.sdf           # the course
└── src/ardupilot_gazebo/                 # pristine upstream clone, gitignored
```

---

## Documentation

### Getting Started
- **[Installation Guide](docs/installation/installation-README.md)** - Install ROS2, MAVROS, ArduPilot SITL
- **[SITL Simulation](docs/guides/SITL_SETUP.md)** - Test missions safely in simulation

### Hardware Deployment  
- **[Complete Workflow](docs/WORKFLOW.md)** - SITL → Bench → Flight testing progression
- **[Real Hardware Setup](docs/guides/REAL_HARDWARE_SETUP.md)** - Deploy on a companion computer + Flight Controller
- **[Mission Planner Connection](docs/guides/MISSION_PLANNER_SETUP.md)** - Ground station monitoring (optional)

### Reference
- **[Mission Examples](scripts/missions/README.md)** - Write autonomous missions in Python
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and solutions

---

## Tested Hardware

This framework has been validated on the following configuration:

### **Primary Test Platform**

| Component | Detail |
|-----------|--------|
| **Flight Controller** | Pixhawk 6C |
| **Connection** | TELEM3 (SERIAL4) UART → companion computer, `/dev/ttyAMA0` @ 921600 |
| **Companion Computer** | Raspberry Pi 4 (8GB) |
| **Firmware** | ArduCopter v4.5.7 |
| **ROS2 Version** | Humble Hawksbill |
| **OS** | Ubuntu 22.04 LTS |

---

### **Compatible Hardware**

| Component | Validated | Notes |
|-----------|-----------|-------|
| **Flight Controllers** | | |
| Pixhawk 6C | ✅ Tested | Primary platform (TELEM3 UART, `/dev/ttyAMA0` @ 921600) |
| **Companion Computers** | | |
| Raspberry Pi 4 (4GB+) | ✅ Tested | Ubuntu 22.04 native |
| Raspberry Pi 5 | ⚠️ Compatible | May require Docker (Ubuntu 22.04 support) |
| Jetson Nano | ⚠️ Compatible | Different architecture, may need adjustments |

**Note:** "Compatible" means theoretically supported but not tested by maintainer. **Some hardware may work without changes, others may require minor modifications** such as serial port names, baud rates, or Docker setup for OS compatibility. Community testing and feedback welcome!

---

## Safety Notice

**Before testing on real hardware:**
- ✅ Remove all propellers during bench testing
- ✅ Secure drone on stable surface
- ✅ Have RC transmitter ready for manual override
- ✅ Read the [Real Hardware Setup Guide](docs/guides/REAL_HARDWARE_SETUP.md)
- ✅ Follow local drone regulations

**Motors WILL spin when armed - propellers off = safe testing!**

---

## Contributing

**This project is actively maintained and welcomes contributions!**

We're building this based on community feedback and real-world usage. Your input helps make this better for everyone.

**Ways to contribute:**
- Report bugs or hardware compatibility issues
- Suggest features or improvements
- Improve documentation
- Test on different hardware configurations
- Share your mission examples

**See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.**

### **Roadmap (Community-Driven)**

Planned improvements based on feedback:
- More mission examples (waypoint patterns, search patterns)
- Additional hardware configurations (Jetson, RPi5)
- Computer vision integration examples
- Multi-drone coordination examples
- Video tutorials


---

## License

---

## Acknowledgments

- ROS2 community
- ArduPilot developers  
- MAVROS maintainers

---



---

If this project helps you, consider giving it a ⭐ on GitHub.

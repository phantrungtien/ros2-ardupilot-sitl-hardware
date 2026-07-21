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

### Fly a Gate Course

The drone flies a **four-ring course** on the onboard camera alone — colour to find
each ring, depth to range it. Nothing tells it where the rings are, in what order, or
when it is done: it maps them, plans through them, flies them, judges each pass, and
goes home.

Seven nodes, three packages:

| # | Node | Does |
|---|------|------|
| N1 | `simtofly_gates/gate_detect` | Finds each orange ring (HSV mask) and fits its **plane** in the depth cloud → `/gate/detections`, in metres |
| N2 | `simtofly_gates/gate_tracker` | Associates detections across frames into stable gates → `/gate/map` |
| N3+N4 | `simtofly_gates/gate_planner` | Builds the occupancy grid, carves a corridor through the target ring, and runs Dijkstra over it → `/planner/path` |
| N5 | `simtofly_gates/trajectory_follower` | Fits a **Mueller minimum-jerk** trajectory to that path and streams position setpoints at 20 Hz |
| N6 | `simtofly_gates/mission_fsm` | Owns the mission: judges each pass, enforces the geofence and pose watchdog, and ends the flight |
| N7 | `simtofly_gates/plan_overlay` | Draws the live plan back onto the colour image → `/plan_overlay/image` |

`gate_order.py` is not a node. It holds the one rule N4 and N6 must never disagree
on — which gate to fly next — because N6 does not tell N4 where to go, it only says
which gates are finished, and both then work the target out for themselves.

**The flight, start to finish:**

1. **Takeoff, then a sweep before flying anything.** The drone climbs through three
   levels and turns a full circle at each. The camera is bolted to the airframe, so a
   ring behind it or above it is simply not in frame at takeoff — on the harder
   course one ring sits *behind* the launch point and another 2.5 m higher than the
   rest. The sweep costs about a minute and is what makes the course knowable at all.
2. **Rings ahead first, the one behind last.** Not nearest-first: that flies the hard
   course as a zig-zag, and each reversal is a ~180° turn — the exact geometry that
   makes a minimum-jerk fit infeasible at any duration. Total distance is the same
   either way (38.8 m vs 38.1 m measured); the point is halving the hairpins.
3. **Fly, judge, repeat**, then **RTL** — see below for why not `LAND`.

**A pass is not "we ended up past the ring".** N6 watches the signed distance along
the gate normal, interpolates the exact plane crossing, and measures the off-axis miss
*there* — checking at the current sample instead gets it wrong in both directions
(clean through then drifting wide reads as a miss; flying outside the rim and lining
up a moment later reads as a pass). A crossing closer than 0.25 m to the rim counts as
a graze, and the gate **stays on the list to be flown again**.

The legacy single-ring `mission_gates` still ships, but it listens to `/gate/obs`,
which nothing has published since N1 was rewritten — it is kept only so the old
behaviour is one argument away.

#### 1. Get the Gazebo world and models

The drone, the rings and the course world live in a **separate repo** — an ArduPilot
Iris carrying a nose-mounted **Intel RealSense D435i** (colour + depth + stereo IR +
IMU), four rings on stands, and the course world:

**→ [phantrungtien/gazebo_ardupilot_env](https://github.com/phantrungtien/gazebo_ardupilot_env)**

```bash
# The launch files expect it at ~/gz_ws — clone it there.
git clone https://github.com/phantrungtien/gazebo_ardupilot_env.git ~/gz_ws
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

Run this from a **desktop terminal** — it opens a `gnome-terminal` window per process
so the logs stay separate, and that needs a real session.

```bash
colcon build --symlink-install
source install/setup.bash

# The whole course, one command: Gazebo, SITL, MAVROS, the depth bridge, the seven
# nodes and RViz. It takes off, sweeps for rings, flies them, judges each pass and
# comes home, drawing the track it flew against the plan it was given.
ros2 launch simtofly_gates gates_mission.launch.py
```

| Argument | Default | What it does |
|----------|---------|--------------|
| `expect_gates` | `4` | How many rings the **course** has. Read the warning below before changing it. |
| `rviz` | `true` | `false` drops the one window that costs real CPU |
| `controller` | `trajectory_follower` | `hold_and_watch` takes off and holds still facing the first ring — for reading topics instead of flying |
| `fsm` | `true` | `false` stops N6 timing gates out and bringing the drone home |

```bash
# Verify N1..N4 without flying the course. Both arguments, or N6 times every gate
# out while the drone sits there and RTLs on you.
ros2 launch simtofly_gates gates_mission.launch.py \
    controller:=hold_and_watch fsm:=false
```

**`expect_gates` is the difference between "finished" and "gave up quietly."**
`RTL: het vong 4/4 xong` grades the drone's map against itself — a ring it never
confirmed is not in `/gate/map`, so it is never flown, never missed, and a three-ring
run reads exactly like a complete one. Nothing on the wire carries the real number;
only you know it, because you placed the rings. Get it wrong and a good run fails, or
a bad one passes.

The launch flies `gates_course.sdf`. The same world repo also ships
`gates_course_hard.sdf` — same four rings, but one sits *behind* the launch point and
one 2.5 m above the others. There is no `world` argument yet: switching means editing
the one `gz sim` line near the top of `gates_mission.launch.py`.

It brings up the `ros_gz_bridge` that carries the depth image into ROS. Colour needs
no bridge — it still arrives as H.264/RTP on UDP 5600.

Watch it work:

```bash
ros2 run rqt_image_view rqt_image_view /plan_overlay/image   # the plan, on the camera
ros2 topic echo /gate/map                                    # the rings it believes in

# Grade a run. Must be running DURING the flight -- it watches the crossings live.
# Ctrl-C to print the scorecard; it exits non-zero if a ring was missed or never seen.
ros2 run simtofly_gates verify_n6 --expect 4
```

**What a healthy run looks like in the logs:**

| Line | Node | Means |
|------|------|-------|
| `quet tang 3/3 xong` | N5 | the opening sweep finished; every ring it can see is seen |
| `muc tieu -> vong N` | N6 | target chosen — the ring behind you should be the LAST one |
| `vong N XONG: LOT (lech truc ...)` | N6 | flown, with the off-axis miss at the plane crossing |
| `RTL: het vong: 4/4 xong` | N6 | course complete — trust it only alongside `expect_gates` |
| `no TF map<-d435i_...` | N2 | **should not appear.** Every one is a dropped sighting |
| `THIEU N vong` | N6 | the course had rings this run never met. The run failed |

#### 3. Two things that are easy to get wrong

**The drone ends up inside the map.** Obstacles are inflated by 0.60 m before
planning, and after threading a ring the drone is sitting inside that ring's inflation
skirt — its own voxel and all 26 neighbours read occupied. Dijkstra force-frees the
*start voxel*, but one free cell in a solid block has no outgoing edge, so the search
fails exactly as it would against a genuinely walled-off map. Left unhandled the
mission stalls after the first ring or two and the FSM abandons the rest of the course
on timeout. The planner therefore clears the volume the drone currently occupies —
it is flying there, so whatever the map says about that volume is wrong.

**Landing where the course ends is landing on a stand.** Every ring sits on a 0.35 m
base plate with a mast, directly below its centre, and the course leaves the drone
hovering a stand-off past the *last* ring — i.e. more or less on top of that plate.
`LAND` descends in place, catches the plate, tips, and ArduCopter cuts the motors with
`Crash: Disarming: AngErr>30`. Finishing the course therefore ends in **RTL**, which
returns to the open ground it took off from. Aborts still use plain `LAND`: with a
dead pose estimate or a geofence breach, flying the length of the course is worse than
coming down where you are.

Every node has an offline self-check that needs no simulator:

```bash
for f in src/*/simtofly_*/*.py; do python3 "$f" --selfcheck 2>/dev/null; done
```

**A green self-check does not mean the nodes start.** They deliberately touch no ROS,
so they never construct a node — a missing message import sits happily behind an
all-green run and then kills the node on its first line in the air, which costs a
whole batch of flights to find out. After adding a subscription or publisher, build
one for real:

```bash
source install/setup.bash
python3 -c "
import rclpy; rclpy.init()
from simtofly_gates import gate_tracker, mission_fsm, trajectory_follower as tf
n = gate_tracker.GateTracker(); print(n.get_name()); n.destroy_node()
_, C = tf._build_node(); n = C(); print(n.get_name()); n.destroy_node()
m = mission_fsm.MissionFSM(); print(m.get_name(), m.expect); m.destroy_node()"
```

> **SITL-verified:** 8 consecutive runs, both worlds, every one flying 4/4 rings with
> no ring missed and no ring dropped from the map — worst off-axis miss 0.066 m
> against a 0.75 m hole. Includes the harder course, where one ring is behind the
> launch point and one is 2.5 m above the rest.

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

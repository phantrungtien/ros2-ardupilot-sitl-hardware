# ROS2-ArduPilot SITL & Hardware Integration

![Cover Image](docs/images/cover.png)

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-Copter%204.x-green)](https://ardupilot.org/)

 > Open-source framework to control ArduPilot drones with ROS2. Test missions in SITL simulation, then deploy to real hardware.



Validated on real flights with Cube Orange flight controller and Raspberry Pi 4.

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
        │   (UDP:14550)     │       │   (/dev/ttyACM0)  │
        │                   │       │                   │
        │   ArduPilot       │       │  Cube Orange      │
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

- **Companion Computer Control** - Command drone autonomously from Raspberry Pi in GUIDED mode
- **SITL & Hardware Testing** - Test missions in simulation, then deploy same code to real drone
- **Flight Validated** - Tested on [FuryVision AAV](https://github.com/sidharthmohannair/Fury-Drone-Project) in autonomous flight
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
# 1. Clone this repository
git clone https://github.com/sidharthmohannair/ros2-ardupilot-sitl-hardware.git
cd ros2-ardupilot-sitl-hardware

# 2. Build packages
colcon build
source install/setup.bash

# 3. Make scripts executable (first time only)
chmod +x launch/*.sh

# 4. Start SITL (Terminal 1)
./launch/start_sitl.sh

# 5. Start MAVROS (Terminal 2)
./launch/start_mavros.sh

# 6. Run autonomous mission (Terminal 3)
source install/setup.bash
python3 scripts/missions/mission_simple.py

# 7. Stop cleanly when done
./launch/stop_all.sh
```

**Watch your drone takeoff in the SITL map!**

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

### Deploy to Real Hardware

**After SITL testing succeeds:**
```bash
# 1. Stop SITL completely
./launch/stop_all.sh

# 2. Connect Cube Orange via USB to Raspberry Pi

# 3. Start MAVROS for hardware (Terminal 1)
./launch/start_mavros_real.sh
# Answer safety prompts

# 4. Run SAME mission (Terminal 2)
source install/setup.bash
python3 scripts/missions/mission_simple.py
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
├── src/                          # ROS2 packages
│   ├── simtofly_mavros_sitl/    # SITL simulation configuration
│   └── simtofly_mavros_real/    # Real hardware configuration
├── scripts/missions/             # Autonomous mission examples
├── launch/                       # Helper scripts (start/stop)
├── docs/                         # Complete documentation
└── README.md                     # You are here
```

---

## Documentation

### Getting Started
- **[Installation Guide](docs/installation/installation-README.md)** - Install ROS2, MAVROS, ArduPilot SITL
- **[SITL Simulation](docs/guides/SITL_SETUP.md)** - Test missions safely in simulation

### Hardware Deployment  
- **[Complete Workflow](docs/WORKFLOW.md)** - SITL → Bench → Flight testing progression
- **[Real Hardware Setup](docs/guides/REAL_HARDWARE_SETUP.md)** - Deploy on Raspberry Pi + Flight Controller
- **[Mission Planner Connection](docs/guides/MISSION_PLANNER_SETUP.md)** - Ground station monitoring (optional)

### Reference
- **[Hardware Platform](docs/HARDWARE-REFERENCE.md)** - FuryVision AAV specifications and test data
- **[Mission Examples](scripts/missions/README.md)** - Write autonomous missions in Python
- **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and solutions

---

## Tested Hardware

This framework has been validated on the following configuration:

### **Primary Test Platform: FuryVision AAV**

**Flight Controller:** Pixhawk 6C 
**Companion Computer:** Raspberry Pi 4 (8GB)  
**Firmware:** ArduCopter v4.5.7  
**ROS2 Version:** Humble Hawksbill  
**OS:** Ubuntu 22.04 LTS




**Why This Platform:**
- ✅ **Fully open source** - CAD files, assembly guide, wiring diagrams available
- ✅ **Flight tested** - 8+ minutes flight time, stable autonomous performance
- ✅ **Well documented** - Complete build and test procedures

---

### **Compatible Hardware**

| Component | Validated | Notes |
|-----------|-----------|-------|
| **Flight Controllers** | | |
| Cube Orange+ | ✅ Tested | Primary platform |
| Cube Orange | ⚠️ Compatible | Should work with same setup |
| Pixhawk 4 | ⚠️ Compatible | Should work with same setup |
| Pixhawk 6C | ⚠️ Compatible | Should work with same setup |
| **Companion Computers** | | |
| Raspberry Pi 4 (4GB+) | ✅ Tested | Ubuntu 22.04 native |
| Raspberry Pi 5 | ⚠️ Compatible | May require Docker (Ubuntu 22.04 support) |
| Jetson Nano | ⚠️ Compatible | Different architecture, may need adjustments |

**Note:** "Compatible" means theoretically supported but not tested by maintainer. **Some hardware may work without changes, others may require minor modifications** such as serial port names, baud rates, or Docker setup for OS compatibility. Community testing and feedback welcome!

**For complete hardware specifications:** [Hardware Reference](docs/HARDWARE-REFERENCE.md)

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

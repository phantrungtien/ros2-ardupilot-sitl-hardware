# Installation Guide

Complete installation guide for ROS2-ArduPilot SITL & Hardware Integration framework.
---

## Quick Navigation

Follow these guides **in order**:

1. **[System Setup](01-SYSTEM-SETUP.md)** - Prepare Ubuntu 22.04
2. **[ROS2 Installation](02-ROS2-INSTALLATION.md)** - Install ROS2 Humble
3. **[MAVROS Installation](03-MAVROS-INSTALLATION.md)** - Install MAVROS + GeographicLib
4. **[ArduPilot SITL](04-ARDUPILOT-SITL.md)** - Install ArduPilot simulator

---

## System Requirements

- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **RAM:** Minimum 4GB (8GB recommended)
- **Disk:** 10GB free space
- **Architecture:** amd64 or arm64
- **Internet:** Required for downloads

---

## Time Estimate

| Step | Time Required |
|------|---------------|
| System Setup | 15-20 minutes |
| ROS2 Installation | 10-15 minutes |
| MAVROS Installation | 5-10 minutes |
| ArduPilot SITL | 30-45 minutes |
| **Total** | **60-90 minutes** |

---

## Installation Flow

```
01-SYSTEM-SETUP.md
    ↓
    Locale configured, system updated
    ↓
02-ROS2-INSTALLATION.md
    ↓
    ROS2 Humble installed
    ↓
03-MAVROS-INSTALLATION.md
    ↓
    MAVROS + GeographicLib ready
    ↓
04-ARDUPILOT-SITL.md
    ↓
    ArduPilot SITL simulator working
    ↓
Ready to use this framework!
```

---

## After Installation

Once all steps complete:

### Clone This Repository

```bash
git clone https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware.git
cd ros2-ardupilot-sitl-hardware
```

### Build Packages

```bash
colcon build
source install/setup.bash
```

### Test in Simulation

**Terminal 1:**
```bash
./launch/start_sitl.sh
```

**Terminal 2:**
```bash
./launch/start_mavros.sh
```

**Terminal 3:**
```bash
source install/setup.bash
ros2 run simtofly_mavros_sitl mission_simple
```

> **Tip:** prefer one command? The all-in-one launch starts Gazebo, SITL,
> MAVROS and the mission together:
> `ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=mission_simple`

---

## Troubleshooting

If you encounter issues:

1. **Check each step completed successfully** - Don't skip steps
2. **See troubleshooting sections** - Each guide has troubleshooting
3. **Review error messages carefully** - Often self-explanatory
4. **Search GitHub Issues:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/issues

---

## Additional Resources

### Official Documentation

- **ROS2 Humble:** https://docs.ros.org/en/humble/
- **MAVROS:** https://github.com/mavlink/mavros
- **ArduPilot:** https://ardupilot.org/

### Detailed Tutorials

- **SimToFly Guide:** https://github.com/simtofly/simtofly-guide
  - Comprehensive beginner-friendly tutorials
  - Step-by-step verification
  - Gazebo integration
  - Video demonstrations

### This Framework

- **Main README:** [../../README.md](../../README.md)
- **SITL Guide:** [../guides/SITL_SETUP.md](../guides/SITL_SETUP.md)
- **Hardware Guide:** [../guides/REAL_HARDWARE_SETUP.md](../guides/REAL_HARDWARE_SETUP.md)

---

## Need Help?

- **Issues:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/issues
- **Discussions:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/discussions

---

**Author:** phantrungtien  
**Repository:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware

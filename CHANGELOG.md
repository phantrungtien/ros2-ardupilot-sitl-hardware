# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-12-13

### Added
- Initial release of ROS2-ArduPilot SITL & Hardware Integration Framework
- `simtofly_mavros_sitl` package for ArduPilot SITL simulation
- `simtofly_mavros_real` package for real hardware (Pixhawk 6C)
- Complete mission script (`mission_simple.py`) for autonomous flight
- Helper scripts for easy startup (`start_mavros.sh`, `start_mavros_real.sh`, `start_sitl.sh`)
- Mission Planner/QGroundControl network integration
- Comprehensive documentation:
  - Real Hardware Setup Guide
  - Mission Planner Connection Guide
  - SITL Simulation Guide
  - Mission Planning Guide
- Safety checks and procedures for hardware testing
- Tested and validated on real hardware (Pixhawk 6C + Raspberry Pi 4)

### Features
- Dual environment support (SITL + Real Hardware)
- Same mission code works for both simulation and real drone
- UDP telemetry forwarding to ground control stations
- Complete safety checklists and emergency procedures
- Production-ready configuration files

### Tested On
- **Hardware:** Pixhawk 6C flight controller
- **Companion Computer:** Raspberry Pi 4 (4GB)
- **ROS2 Version:** Humble Hawksbill
- **ArduPilot:** ArduCopter 4.x
- **OS:** Ubuntu 22.04

---

## Future Releases

### Planned for v1.1.0
- Additional mission examples (waypoint missions, survey patterns)
- Parameter optimization guide
- Performance tuning documentation
- Multi-drone support

### Planned for v1.2.0
- Computer vision integration examples
- Object detection and tracking
- Obstacle avoidance demonstrations

---

**Format:**
- `Added` - New features
- `Changed` - Changes in existing functionality
- `Deprecated` - Soon-to-be removed features
- `Removed` - Removed features
- `Fixed` - Bug fixes
- `Security` - Security improvements

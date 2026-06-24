# Complete Testing Workflow

**How to test missions safely: SITL first, then real hardware.**


---

## Overview

This framework supports **both simulation and real hardware** on the **same Raspberry Pi**:

1. **SITL Mode** - Test missions in software simulation (no drone needed)
2. **Hardware Mode** - Deploy same missions on real drone

**Key advantage:** Develop and test on Raspberry Pi in SITL, then switch to hardware without changing code!

---

## Helper Scripts Reference

### Simulation (SITL)

| Script | Purpose |
|--------|---------|
| `./launch/start_sitl.sh` | Start ArduPilot SITL simulator |
| `./launch/start_mavros.sh` | Connect MAVROS to SITL (UDP) |
| `./launch/stop_sitl.sh` | Stop SITL only |
| `./launch/stop_mavros.sh` | Stop MAVROS only |
| `./launch/stop_all.sh` | **Stop both SITL and MAVROS cleanly** |

### Real Hardware

| Script | Purpose |
|--------|---------|
| `./launch/start_mavros_real.sh` | Connect MAVROS to Cube Orange (Serial) |

**Note:** Use `Ctrl+C` to stop real hardware MAVROS.

---

## Workflow 1: SITL Testing (Simulation)

### Prerequisites

- ✅ Raspberry Pi with this repository installed
- ✅ ArduPilot SITL installed
- ✅ ROS2 Humble + MAVROS installed

### Step-by-Step

**Terminal 1: Start SITL**
```bash
cd ~/ros2-ardupilot-sitl-hardware
./launch/start_sitl.sh
```

**Wait for:** `STABILIZE>` prompt

---

**Terminal 2: Start MAVROS (UDP connection)**
```bash
cd ~/ros2-ardupilot-sitl-hardware
./launch/start_mavros.sh
```

**Wait for:** `CON: Got HEARTBEAT, connected`

---

**Terminal 3: Run Mission**
```bash
cd ~/ros2-ardupilot-sitl-hardware
source install/setup.bash
python3 scripts/missions/mission_simple.py
```

**Watch:** Mission executes in SITL

---

**When Done: Clean Shutdown**
```bash
# Terminal 4 (or after Ctrl+C on mission)
cd ~/ros2-ardupilot-sitl-hardware
./launch/stop_all.sh
```

**This ensures:**
- SITL processes killed cleanly
- MAVROS stopped properly
- No zombie processes
- Ready for next test

---

## Workflow 2: Real Hardware Testing

### Prerequisites

- ✅ SITL testing completed successfully
- ✅ Mission works in simulation
- ✅ Cube Orange connected via USB to Raspberry Pi
- ✅ **Propellers REMOVED** (bench testing)
- ✅ Battery connected, Cube Orange powered
- ✅ RC transmitter ON

### Step-by-Step

**Ensure SITL is stopped:**
```bash
./launch/stop_all.sh
```

**Verify Cube Orange detected:**
```bash
ls -l /dev/ttyACM0
# Should show device
```

---

**Terminal 1: Start MAVROS for Real Hardware**
```bash
cd ~/ros2-ardupilot-sitl-hardware
./launch/start_mavros_real.sh
```

**Script will ask:**
1. Safety checks confirmed? → `y`
2. Send telemetry to Mission Planner? → `y` or `n`
3. If yes, enter laptop IP (default: 10.73.2.236)

**Wait for:** `CON: Got HEARTBEAT, connected. FCU: ArduPilot`

---

**Terminal 2: Run Same Mission**
```bash
cd ~/ros2-ardupilot-sitl-hardware
source install/setup.bash
python3 scripts/missions/mission_simple.py
```

**Expected behavior:**
- Sets GUIDED mode
- Arms motors (**props must be OFF!**)
- Attempts takeoff (motors spin, no flight without props)
- Sends position commands
- RTL mode

**To stop:** `Ctrl+C` in Terminal 1

---

## Workflow 3: Switch Between SITL and Hardware

### Scenario: Testing Mission Changes

**1. Test in SITL first:**
```bash
# Start SITL
./launch/start_sitl.sh          # Terminal 1
./launch/start_mavros.sh         # Terminal 2

# Run mission
python3 scripts/missions/mission_simple.py  # Terminal 3

# Stop cleanly
./launch/stop_all.sh             # Terminal 4
```

**2. Switch to hardware:**
```bash
# Verify SITL stopped
./launch/stop_all.sh

# Connect hardware
./launch/start_mavros_real.sh    # Terminal 1

# Run SAME mission
python3 scripts/missions/mission_simple.py  # Terminal 2

# Stop
Ctrl+C in Terminal 1
```

**3. Back to SITL for more testing:**
```bash
# Ensure hardware disconnected
Ctrl+C (if still running)

# Restart SITL
./launch/start_sitl.sh           # Terminal 1
./launch/start_mavros.sh          # Terminal 2

# Continue development
python3 scripts/missions/mission_simple.py  # Terminal 3
```

---

## Common Issues

### SITL won't start after hardware test

**Cause:** MAVROS still running from hardware session

**Solution:**
```bash
./launch/stop_all.sh
# Wait 2 seconds
./launch/start_sitl.sh
```

---

### Port already in use

**Cause:** Previous SITL/MAVROS not stopped cleanly

**Solution:**
```bash
./launch/stop_all.sh

# Verify nothing running
ps aux | grep -E "sim_vehicle|mavros_node"
# Should show nothing (except grep itself)
```

---

### Hardware connection fails

**Cause:** SITL still running

**Solution:**
```bash
./launch/stop_all.sh
# Ensure /dev/ttyACM0 exists
ls -l /dev/ttyACM0
./launch/start_mavros_real.sh
```

---

### Mission behaves differently in hardware vs SITL

**Expected!** This is normal:

**SITL:**
- Perfect sensors
- No GPS drift
- Instant response
- No wind

**Hardware:**
- Real sensor noise
- GPS lock variations
- Mechanical delays
- Environmental factors

**Always test in SITL first, but expect differences on real hardware.**

---

## Best Practices

### SITL Testing

1. ✅ Test all mission logic in SITL first
2. ✅ Verify mode changes work
3. ✅ Check altitude holds correctly
4. ✅ Test RTL behavior
5. ✅ Use `stop_all.sh` for clean shutdowns

### Hardware Testing

1. ✅ **Always remove propellers** for bench testing
2. ✅ Stop SITL completely first
3. ✅ Verify USB connection (`ls /dev/ttyACM0`)
4. ✅ Check battery voltage sufficient
5. ✅ Have RC transmitter ready for override
6. ✅ Start with simple missions
7. ✅ Test mode changes manually first

### Development Cycle

```
Write Mission
    ↓
Test in SITL (repeat until works)
    ↓
Bench Test Hardware (props OFF!)
    ↓
Outdoor Test (props ON, manual flight first)
    ↓
Autonomous Flight (start simple)
```

---

## Mission Code Compatibility

### Same Code, Both Environments

The mission scripts work identically in SITL and hardware because:

✅ **Same ROS2 topics:**
- `/mavros/state`
- `/mavros/local_position/pose`
- `/mavros/cmd/arming`
- `/mavros/set_mode`

✅ **Same MAVROS services:**
- `/mavros/cmd/arming`
- `/mavros/set_mode`
- `/mavros/cmd/takeoff`

✅ **Connection abstracted:**
- SITL: UDP connection (`udp://:14550@`)
- Hardware: Serial connection (`/dev/ttyACM0:921600`)
- Mission code doesn't care!

### No Code Changes Needed

```python
# This works in SITL:
python3 scripts/missions/mission_simple.py

# Same code works on hardware:
python3 scripts/missions/mission_simple.py
```

**Only difference:** Which MAVROS you start (UDP vs Serial)

---

## Summary Cheatsheet

### Quick Commands

**SITL Testing:**
```bash
./launch/start_sitl.sh           # Terminal 1
./launch/start_mavros.sh          # Terminal 2
python3 scripts/missions/mission_simple.py  # Terminal 3
./launch/stop_all.sh              # When done
```

**Hardware Testing:**
```bash
./launch/stop_all.sh              # Ensure SITL stopped
./launch/start_mavros_real.sh     # Terminal 1 (answer prompts)
python3 scripts/missions/mission_simple.py  # Terminal 2
Ctrl+C                            # When done
```

**Emergency Stop:**
```bash
./launch/stop_all.sh
# Or
pkill -9 mavros_node
pkill -9 -f sim_vehicle
```

---

## Next Steps

- **New to this?** Start with [SITL Setup Guide](guides/SITL_SETUP.md)
- **Ready for hardware?** See [Real Hardware Setup](guides/REAL_HARDWARE_SETUP.md)
- **Creating missions?** Check [Mission Examples](../scripts/missions/README.md)

---

**Author:** phantrungtien  
**Repository:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware

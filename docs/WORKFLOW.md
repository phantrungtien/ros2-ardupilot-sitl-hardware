# Complete Testing Workflow

**How to test missions safely: SITL first, then real hardware.**


---

## Overview

This framework supports **both simulation and real hardware** on the **same companion computer**:

1. **SITL Mode** - Test missions in software simulation (no drone needed)
2. **Hardware Mode** - Deploy same missions on real drone

**Key advantage:** Develop and test on the companion computer in SITL, then switch to hardware without changing code!

---

## Helper Scripts Reference

### Simulation (SITL)

| Script | Purpose |
|--------|---------|
| `./launch/start_sitl.sh` | Start ArduPilot SITL simulator |
| `./launch/start_mavros.sh` | Connect MAVROS to SITL (UDP) |

To stop everything, kill the processes directly:

```bash
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
```

That one line covers Gazebo, SITL, MAVROS and the mission/vision nodes. Ctrl+C
in a window stops only that window's program; the pkill also clears strays left
holding UDP 14550 / 5600, which is what blocks the next run.

### Real Hardware

| Script | Purpose |
|--------|---------|
| `./launch/start_mavros_real.sh` | Connect MAVROS to Pixhawk 6C (TELEM3 UART) |

**Note:** Use `Ctrl+C` to stop real hardware MAVROS.

---

## Workflow 1: SITL Testing (Simulation)

### Prerequisites

- ✅ Companion computer with this repository installed
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
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
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
- ✅ Pixhawk 6C wired to the companion computer (TELEM3 UART)
- ✅ **Propellers REMOVED** (bench testing)
- ✅ Battery connected, Pixhawk 6C powered
- ✅ RC transmitter ON

### Step-by-Step

**Ensure SITL is stopped:**
```bash
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
```

**Verify the UART device exists:**
```bash
ls -l /dev/ttyAMA0
# Should show device (crw-rw---- root dialout)
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
# Stop cleanly (Terminal 4)
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
```

**2. Switch to hardware:**
```bash
# Verify SITL stopped
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'

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
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
# Wait 2 seconds
./launch/start_sitl.sh
```

---

### Port already in use

**Cause:** Previous SITL/MAVROS not stopped cleanly

**Solution:**
```bash
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'

# Verify nothing running
ps aux | grep -E "sim_vehicle|mavros_node"
# Should show nothing (except grep itself)
```

---

### Hardware connection fails

**Cause:** SITL still running

**Solution:**
```bash
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
# Ensure /dev/ttyAMA0 exists
ls -l /dev/ttyAMA0
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
5. ✅ Kill leftover processes (see the pkill line above) for clean shutdowns

### Hardware Testing

1. ✅ **Always remove propellers** for bench testing
2. ✅ Stop SITL completely first
3. ✅ Verify UART device (`ls /dev/ttyAMA0`)
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
- Hardware: Serial UART connection (`/dev/ttyAMA0:921600`)
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
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'              # When done
```

**Hardware Testing:**
```bash
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'              # Ensure SITL stopped
./launch/start_mavros_real.sh     # Terminal 1 (answer prompts)
python3 scripts/missions/mission_simple.py  # Terminal 2
Ctrl+C                            # When done
```

**Emergency Stop:**
```bash
pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
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

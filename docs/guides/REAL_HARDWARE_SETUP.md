# Real Hardware Setup Guide - Cube Orange + Raspberry Pi

**Author:** Sidharth Mohan Nair
**GitHub:** https://github.com/sidharthmohannair

Complete guide to configure and test your real drone with ROS2 MAVROS.

---

## ⚠️ CRITICAL SAFETY WARNING

**BEFORE doing ANY hardware testing:**

1. ✅ **REMOVE ALL PROPELLERS** - This is MANDATORY
2. ✅ Secure drone on stable bench/table
3. ✅ Clear area of people and objects
4. ✅ Have RC transmitter ON and ready
5. ✅ Know how to quickly disconnect battery
6. ✅ Never test indoors with propellers attached

**Motors WILL spin when armed - props off = safe testing!**

---

## Hardware Setup

### Equipment Needed

- **Flight Controller:** Cube Orange (or compatible Pixhawk)
- **Companion Computer:** Raspberry Pi (already configured with ROS2 Humble)
- **Connection:** USB cable (Cube Orange USB port → RPi USB port)
- **Power:** Battery or power module for Cube Orange
- **RC Transmitter:** Powered ON with good signal
- **Laptop:** For initial parameter configuration (Mission Planner/QGC)

---

## STEP 1: Configure Cube Orange Parameters

### Connect to Mission Planner or QGroundControl

**Using Mission Planner:**

1. Connect Cube Orange to laptop via USB
2. Open Mission Planner → Connect
3. Go to `CONFIG/TUNING` → `Full Parameter List`
4. Set these parameters:

```
SERIAL0_PROTOCOL = 2        (MAVLink2 on USB)
SERIAL0_BAUD = 921600       (921600 baud rate)
```

5. Click **"Write Params"**
6. **Reboot** Cube Orange
7. Reconnect and verify parameters saved

**Using QGroundControl:**

1. Connect vehicle via USB
2. Go to `Vehicle Setup` → `Parameters`
3. Search `SERIAL0_PROTOCOL` → Set to `2`
4. Search `SERIAL0_BAUD` → Set to `921600`
5. **Reboot** vehicle
6. Verify settings saved

### Why These Settings?

- `SERIAL0_PROTOCOL = 2`: Enables MAVLink v2 on USB port
- `SERIAL0_BAUD = 921600`: High-speed communication (921.6 kbps)
- Without these, USB port won't output proper MAVLink data

---

## STEP 2: Setup USB Connection on Raspberry Pi

### Physical Connection

1. **Power ON** Cube Orange (connect battery or power module)
2. **Connect USB cable:** Cube Orange USB → Raspberry Pi USB port
3. Wait for Cube Orange to boot (LEDs, beeps)

### Verify Device Appears

```bash
# List serial devices
ls -l /dev/tty* | grep -E "ACM|USB"

# Should see one of:
# /dev/ttyACM0  (most common for Cube Orange)
# /dev/ttyUSB0  (if using USB-serial adapter)
```

**Check kernel messages:**

```bash
dmesg | tail -20
```

**Expected output:**
```
[  123.456] usb 1-1: new full-speed USB device
[  123.567] cdc_acm 1-1:1.0: ttyACM0: USB ACM device
```

**If device doesn't appear:**
- Try different USB cable (some are power-only)
- Try different USB port on RPi
- Verify Cube Orange is powered and booted
- Check USB port on Cube Orange is clean

---

### Set Proper Permissions

Serial ports require proper permissions:

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Verify addition
groups

# ⚠️ MUST REBOOT for changes to take effect
sudo reboot
```

**After reboot, verify:**

```bash
# Check groups
groups
# Should show: ... dialout ...

# Check device permissions
ls -l /dev/ttyACM0
# Output: crw-rw---- 1 root dialout ... /dev/ttyACM0
```

---

### (Optional) Create Persistent Device Name

If `/dev/ttyACM0` changes to `/dev/ttyACM1` etc. on reboot, create a udev rule:

```bash
# Find USB device ID
lsusb

# Create udev rule
sudo nano /etc/udev/rules.d/99-cube-orange.rules
```

**Add this line (replace VID:PID with your values):**
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0032", SYMLINK+="cube_orange", MODE="0666"
```

**Reload rules:**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Now you can use `/dev/cube_orange` instead of `/dev/ttyACM0`!

---

### Test Serial Communication

```bash
# Install minicom if needed
sudo apt install minicom -y

# Open serial connection
minicom -D /dev/ttyACM0 -b 921600
```

**What you should see:**
- Binary data/gibberish (MAVLink packets)
- Intermittent characters flowing

**Press Ctrl+A, then X to exit**

✅ **If you see data:** Serial connection works!
❌ **If no data:** Check Cube Orange parameters and power

---

## STEP 3: Launch MAVROS for Real Hardware

### Your New Package Structure

```
~/ros2-ardupilot-sitl-hardware/
├── src/
│   ├── simtofly_mavros_sitl/          # Simulation config
│   │   └── launch/
│   │       └── ardupilot_sitl.launch.py
│   └── simtofly_mavros_real/          # Real hardware config
│       └── launch/
│           └── pixhawk6c.launch.py
├── scripts/                      ← Mission scripts work for both!
├── launch/
│   ├── start_mavros.sh          ← For SITL
│   ├── start_mavros_real.sh     ← For real hardware
│   ├── start_sitl.sh
│   └── stop_all.sh
```

### Start MAVROS for Real Hardware

**Terminal 1:**

```bash
cd ~/ros2-ardupilot-sitl-hardware
./launch/start_mavros_real.sh
```

**OR manually:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2-ardupilot-sitl-hardware/install/setup.bash
ros2 launch simtofly_mavros_real pixhawk6c.launch.py
```

---

### Success Indicators

**You should see:**

```
[INFO] [mavros]: FCU URL: /dev/ttyACM0:921600
[INFO] [mavros]: Starting MAVROS...
[INFO] [mavros]: CON: Got HEARTBEAT, connected. FCU: ArduPilot
[INFO] [mavros]: FCU: ArduCopter V4.x.x
[INFO] [mavros]: VER: 1.1: Capabilities 0x...
[INFO] [mavros]: Built-in MAVLink package version: ...
```

✅ **Key success messages:**
- `Got HEARTBEAT, connected`
- `FCU: ArduPilot` or `FCU: ArduCopter`
- `VER: 1.1: Capabilities`

---

### Troubleshooting Connection Issues

**Error: `Failed to open /dev/ttyACM0`**
- **Cause:** Permission denied
- **Fix:** User not in dialout group, reboot required

**Error: `Connection timeout`**
- **Cause:** No MAVLink data on serial port
- **Fix:** Check Cube Orange `SERIAL0_PROTOCOL = 2`

**Error: `Device not found`**
- **Cause:** USB cable not connected or bad cable
- **Fix:** Check physical connection, try different cable

**No error but no HEARTBEAT:**
- Check Cube Orange is powered ON
- Check correct baud rate (921600)
- Try rebooting Cube Orange

---

## STEP 4: Verify ROS2 Topics

**Terminal 2:**

```bash
source /opt/ros/humble/setup.bash

# List all MAVROS topics
ros2 topic list | grep mavros

# Check connection state
ros2 topic echo /mavros/state --once
```

**Expected output:**
```yaml
connected: true
armed: false
guided: false
manual_input: false
mode: "STABILIZE"  # or whatever current mode
system_status: 3
```

✅ **`connected: true`** = Success!

---

### Monitor Key Topics

```bash
# Check battery voltage
ros2 topic echo /mavros/battery

# Check GPS (if outdoors with GPS lock)
ros2 topic echo /mavros/global_position/global

# Check IMU data
ros2 topic echo /mavros/imu/data

# Monitor altitude
ros2 topic echo /mavros/local_position/pose
```

---

## STEP 5: Bench Test (PROPS OFF!)

### ⚠️ FINAL SAFETY CHECK

Before proceeding:

- [ ] All propellers REMOVED
- [ ] Drone secured on bench
- [ ] RC transmitter ON
- [ ] Clear area around drone
- [ ] Battery connected and Cube Orange powered
- [ ] MAVROS running and connected

---

### Test Flight Modes

**In a new terminal:**

```bash
source /opt/ros/humble/setup.bash

# Test setting GUIDED mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# Check mode changed
ros2 topic echo /mavros/state --once
```

**Response should show:**
```yaml
mode_sent: true
```

**And state should show:**
```yaml
mode: "GUIDED"
```

---

### Test Arming (PROPS MUST BE OFF!)

```bash
# Arm the vehicle
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

**Expected behavior:**
- ⚠️ **Motors MAY start spinning** (this is why props are OFF!)
- Cube Orange LEDs change (solid vs blinking)
- RC transmitter may beep/alert

**Disarm immediately:**

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

**If arming fails, check:**
- GPS lock (if outdoors)
- Pre-arm checks in Mission Planner
- RC transmitter failsafe settings
- Cube Orange errors in MAVROS output

---

## STEP 6: Run Mission Script (BENCH TEST)

Your mission scripts from SITL will work on real hardware!

**⚠️ CRITICAL: Props must still be OFF for bench testing**

```bash
cd ~/ros2-ardupilot-sitl-hardware
source /opt/ros/humble/setup.bash
python3 scripts/missions/mission_simple.py
```

**What should happen:**
1. Script connects to MAVROS ✅
2. Sets GUIDED mode ✅
3. Arms vehicle ✅
4. **Sends takeoff command** - motors will spin but won't takeoff (no props)
5. Sends position commands
6. RTL mode

**This verifies:**
- Communication working
- Commands being accepted
- Mission logic functioning

---

## Differences Between SITL and Real Hardware

| Aspect | SITL | Real Hardware |
|--------|------|---------------|
| **Connection** | UDP (`udp://:14550@`) | Serial (`/dev/ttyACM0:921600`) |
| **Launch File** | `ardupilot_sitl.launch.py` | `pixhawk6c.launch.py` |
| **Package** | `simtofly_mavros_sitl` | `simtofly_mavros_real` |
| **Start Script** | `./start_mavros.sh` | `./start_mavros_real.sh` |
| **Mission Scripts** | ✅ Same scripts work! | ✅ Same scripts work! |
| **Safety** | Simulated, safe | **REMOVE PROPS!** |

---

## Next Steps: Outdoor Flight Test

**When ready for actual flight:**

1. ✅ Bench tests successful
2. ✅ All systems verified
3. ✅ **Reinstall propellers** (correct direction!)
4. ✅ Go to open outdoor area (GPS lock required)
5. ✅ Follow local drone regulations
6. ✅ Start with manual RC flight first
7. ✅ Test autonomous mode in small increments
8. ✅ Always have RC override ready

---

## Emergency Procedures

### If Something Goes Wrong

**During bench test:**
1. Press `Ctrl+C` in script terminal
2. Disconnect battery immediately
3. Check error messages in MAVROS output

**During flight:**
1. **Switch to STABILIZE mode on RC** (manual control)
2. Land manually using RC
3. Review telemetry logs

### Kill Switch

Always have RC transmitter ready to:
- Switch to manual mode (STABILIZE/ALT_HOLD)
- Disarm via RC switch
- Emergency land

---

## Useful Commands

### Check Connection Status
```bash
ros2 topic echo /mavros/state
```

### Monitor Position
```bash
ros2 topic echo /mavros/local_position/pose
```

### Check Battery
```bash
ros2 topic echo /mavros/battery
```

### Set Flight Mode
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

### Arm/Disarm
```bash
# Arm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Disarm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

---

## Troubleshooting Common Issues

### MAVROS won't start
- Check device exists: `ls -l /dev/ttyACM0`
- Check permissions: `groups` (should show dialout)
- Reboot if just added to dialout group

### Connected but no topics updating
- Check Cube Orange parameters (SERIAL0_PROTOCOL)
- Verify baud rate matches (921600)
- Check USB cable quality

### Can't arm vehicle
- Check GPS lock (outdoors)
- Review pre-arm checks in Mission Planner
- Check battery voltage sufficient
- Verify RC transmitter connected

---

## Package Structure

```
~/ros2-ardupilot-sitl-hardware/
├── src/
│   ├── simtofly_mavros_sitl/          # Simulation config
│   │   └── launch/
│   │       └── ardupilot_sitl.launch.py
│   └── simtofly_mavros_real/          # Real hardware config
│       └── launch/
│           └── pixhawk6c.launch.py
├── scripts/
│   └── missions/
│       ├── mission_simple.py        # Works for both SITL and real!
│       └── README.md
├── launch/
│   ├── start_sitl.sh                # Start simulation
│   ├── start_mavros.sh              # MAVROS for SITL
│   ├── start_mavros_real.sh         # MAVROS for real hardware
│   └── stop_all.sh                  # Stop SITL + MAVROS
└── docs/
    └── guides/
        └── REAL_HARDWARE_SETUP.md   # This file
```

---

## Summary Checklist

**Before first connection:**
- [ ] Cube Orange parameters set (`SERIAL0_PROTOCOL=2`, `SERIAL0_BAUD=921600`)
- [ ] User added to dialout group + rebooted
- [ ] USB cable connected
- [ ] `simtofly_mavros_real` package built

**Before each test:**
- [ ] Propellers REMOVED (bench testing)
- [ ] Drone secured
- [ ] RC transmitter ON
- [ ] Battery connected
- [ ] Clear area

**Connection test:**
- [ ] MAVROS starts without errors
- [ ] `connected: true` in `/mavros/state`
- [ ] Topics publishing data
- [ ] Can set flight modes via ROS2

**Bench test:**
- [ ] Can arm/disarm
- [ ] Motors respond (props OFF!)
- [ ] Mission script runs
- [ ] Commands accepted

---

**Author:** Sidharth Mohan Nair
**GitHub:** https://github.com/sidharthmohannair

**Stay safe and happy flying!** 🚁

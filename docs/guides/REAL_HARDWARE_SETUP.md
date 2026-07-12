# Real Hardware Setup Guide - Pixhawk 6C + Companion Computer

**Author:** phantrungtien
**GitHub:** https://github.com/phantrungtien

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

- **Flight Controller:** Pixhawk 6C (or compatible Pixhawk/Cube)
- **Companion Computer:** any embedded Linux board with ROS2 Humble (validated on Raspberry Pi 4; Jetson works with adjusted port names)
- **Connection:** TELEM3 (SERIAL4) UART ↔ companion computer GPIO UART (TX/RX/GND)
- **Power:** Battery or power module for the Pixhawk 6C
- **RC Transmitter:** Powered ON with good signal
- **Laptop:** For initial parameter configuration (Mission Planner/QGC)

> **Why a serial UART link, not USB?** This setup connects the Pixhawk's
> **TELEM3 (SERIAL4)** port to the companion computer's GPIO UART
> (`/dev/ttyAMA0`), freeing the USB port and matching the flight wiring. If you
> prefer USB (`/dev/ttyACM0`) for bench work, change `fcu_url` accordingly and
> configure `SERIAL0_*` instead of `SERIAL4_*`.

---

## STEP 1: Configure Pixhawk 6C Parameters

### Connect to Mission Planner or QGroundControl

**Using Mission Planner:**

1. Connect Pixhawk 6C to laptop via USB
2. Open Mission Planner → Connect
3. Go to `CONFIG/TUNING` → `Full Parameter List`
4. Set these parameters (TELEM3 = SERIAL4):

```
SERIAL4_PROTOCOL = 2        (MAVLink2 on TELEM3)
SERIAL4_BAUD = 921600       (921600 baud rate)
```

5. Click **"Write Params"**
6. **Reboot** the Pixhawk 6C
7. Reconnect and verify parameters saved

**Using QGroundControl:**

1. Connect vehicle via USB
2. Go to `Vehicle Setup` → `Parameters`
3. Search `SERIAL4_PROTOCOL` → Set to `2`
4. Search `SERIAL4_BAUD` → Set to `921600`
5. **Reboot** vehicle
6. Verify settings saved

### Why These Settings?

- `SERIAL4_PROTOCOL = 2`: Enables MAVLink v2 on the TELEM3 port
- `SERIAL4_BAUD = 921600`: High-speed communication (921.6 kbps)
- Without these, TELEM3 won't output proper MAVLink data

> **Note on baud:** `921600` over GPIO UART works on short, clean wiring. If you
> see dropped/garbled data on a long or noisy link, drop both sides to `57600`
> (`SERIAL4_BAUD = 57600` and `/dev/ttyAMA0:57600`).

---

## STEP 2: Wire and Enable the GPIO UART on the Companion Computer

### Physical Connection (TELEM3 ↔ GPIO UART)

Wire three lines between the Pixhawk TELEM3 connector and the companion
computer's UART pins (cross TX↔RX):

| Pixhawk TELEM3 | → | Companion computer UART |
|----------------|---|--------------------------|
| TX             | → | RX                       |
| RX             | → | TX                       |
| GND            | → | GND                      |

> **Raspberry Pi pinout (example):** TELEM3 TX → RPi RX (GPIO15, pin 10),
> TELEM3 RX → RPi TX (GPIO14, pin 8), GND → GND. **Do NOT connect the 5V line.**
> On a Jetson the UART pins and device name differ (see the note in STEP 3).

1. **Power ON** the Pixhawk 6C (connect battery or power module)
2. **Connect the three UART wires** as above
3. Wait for the Pixhawk to boot (LEDs, beeps)

### Enable the UART (Raspberry Pi example)

The Raspberry Pi's primary GPIO UART (`/dev/ttyAMA0`) is used by Bluetooth and
the serial console by default — both must be freed:

```bash
# Edit boot config
sudo nano /boot/firmware/config.txt   # (older images: /boot/config.txt)
```

Add / ensure these lines:

```
enable_uart=1
dtoverlay=disable-bt
```

Then disable the serial login console so MAVROS owns the port:

```bash
sudo raspi-config        # Interface Options → Serial Port:
                         #   login shell over serial = NO
                         #   serial hardware enabled = YES
sudo systemctl disable hciuart      # release UART from Bluetooth
sudo reboot
```

> **Other companion computers:** the device name and enable procedure differ —
> e.g. a Jetson exposes its UART as `/dev/ttyTHS1` (configured via its own
> pinmux/jetson-io tools), not `/dev/ttyAMA0`. Use that device in every command
> below.

---

### Verify the UART Device Exists

Unlike a USB FCU, the GPIO UART is a built-in device — it is always present once
enabled (no hot-plug, no `dmesg` USB event):

```bash
ls -l /dev/ttyAMA0
# Expected: crw-rw---- 1 root dialout ... /dev/ttyAMA0
```

**If the device is missing or still owned by Bluetooth:**
- Re-check `enable_uart=1` and `dtoverlay=disable-bt` in the boot config
- Confirm the serial login console is disabled
- Reboot and check again

---

### Set Proper Permissions

Serial ports require the user to be in the `dialout` group:

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Verify addition
groups

# ⚠️ MUST REBOOT (or re-login) for changes to take effect
sudo reboot
```

**After reboot, verify:**

```bash
# Check groups
groups
# Should show: ... dialout ...

# Check device permissions
ls -l /dev/ttyAMA0
# Output: crw-rw---- 1 root dialout ... /dev/ttyAMA0
```

> **No udev rule needed:** the GPIO UART has a fixed name (`/dev/ttyAMA0`), so it
> never renumbers across reboots the way a USB `/dev/ttyACM*` device can.

---

### Test Serial Communication

```bash
# Install minicom if needed
sudo apt install minicom -y

# Open serial connection
minicom -D /dev/ttyAMA0 -b 921600
```

**What you should see:**
- Binary data/gibberish (MAVLink packets)
- Intermittent characters flowing

**Press Ctrl+A, then X to exit**

✅ **If you see data:** Serial connection works!
❌ **If no data:** Check Pixhawk parameters (`SERIAL4_*`), wiring (TX↔RX crossed), and power

---

## STEP 3: Launch MAVROS for Real Hardware

### Your Package Structure

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
│   └── start_sitl.sh
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

> Both default to `fcu_url:=/dev/ttyAMA0:921600`. On a different companion
> computer, override it, e.g.
> `ros2 launch simtofly_mavros_real pixhawk6c.launch.py fcu_url:=/dev/ttyTHS1:921600`.

---

### Success Indicators

**You should see:**

```
[INFO] [mavros]: FCU URL: /dev/ttyAMA0:921600
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

**Error: `Failed to open /dev/ttyAMA0`**
- **Cause:** Permission denied, or UART still owned by Bluetooth/console
- **Fix:** User not in dialout group (reboot required); confirm `dtoverlay=disable-bt` and serial console disabled

**Error: `Connection timeout`**
- **Cause:** No MAVLink data on the UART
- **Fix:** Check Pixhawk `SERIAL4_PROTOCOL = 2` and that TX↔RX are crossed correctly

**Error: `Device not found`**
- **Cause:** UART not enabled in the boot config
- **Fix:** Set `enable_uart=1` + `dtoverlay=disable-bt`, reboot

**No error but no HEARTBEAT:**
- Check the Pixhawk 6C is powered ON
- Check correct baud rate (921600 on both sides)
- Verify GND is connected and TX/RX are not swapped
- Try rebooting the Pixhawk 6C

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
- [ ] Battery connected and Pixhawk 6C powered
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
- Pixhawk 6C LEDs change (solid vs blinking)
- RC transmitter may beep/alert

**Disarm immediately:**

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

**If arming fails, check:**
- GPS lock (if outdoors)
- Pre-arm checks in Mission Planner
- RC transmitter failsafe settings
- Pixhawk 6C errors in MAVROS output

---

## STEP 6: Run Mission Script (BENCH TEST)

Your mission scripts from SITL will work on real hardware!

**⚠️ CRITICAL: Props must still be OFF for bench testing**

```bash
cd ~/ros2-ardupilot-sitl-hardware
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run simtofly_mavros_sitl mission_simple
```

**What should happen:**
1. Script connects to MAVROS ✅
2. Sets GUIDED mode ✅
3. Arms vehicle (retries until prearm/EKF/GPS ready) ✅
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
| **Connection** | UDP (`udp://:14550@`) | Serial UART (`/dev/ttyAMA0:921600`) |
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
- Check device exists: `ls -l /dev/ttyAMA0`
- Check permissions: `groups` (should show dialout)
- Reboot if just added to dialout group

### Connected but no topics updating
- Check Pixhawk parameters (`SERIAL4_PROTOCOL = 2`)
- Verify baud rate matches (921600 on both sides)
- Check wiring quality (TX↔RX crossed, common GND)

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
│   └── start_mavros_real.sh         # MAVROS for real hardware
└── docs/
    └── guides/
        └── REAL_HARDWARE_SETUP.md   # This file
```

---

## Summary Checklist

**Before first connection:**
- [ ] Pixhawk 6C parameters set (`SERIAL4_PROTOCOL=2`, `SERIAL4_BAUD=921600`)
- [ ] TELEM3 ↔ companion UART wired (TX↔RX crossed, common GND)
- [ ] Companion UART enabled + freed from Bluetooth/console
- [ ] User added to dialout group + rebooted
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

**Author:** phantrungtien
**GitHub:** https://github.com/phantrungtien

**Stay safe and happy flying!** 🚁

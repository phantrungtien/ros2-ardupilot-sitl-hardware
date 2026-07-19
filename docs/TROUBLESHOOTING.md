# Troubleshooting Guide

Common issues and solutions for ROS2-ArduPilot integration.

---

## Build Issues

### Package not found after colcon build

**Symptoms:**
```
Package 'simtofly_mavros_sitl' not found
```

**Solution:**
```bash
cd ~/ros2-ardupilot-sitl-hardware
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

### Python import errors

**Symptoms:**
```
ModuleNotFoundError: No module named 'rclpy'
```

**Solution:**
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2-ardupilot-sitl-hardware/install/setup.bash
```

---

## SITL Issues

### sim_vehicle.py not found

**Symptoms:**
```
bash: sim_vehicle.py: command not found
```

**Solution:**
```bash
# Add ArduPilot tools to PATH
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
source ~/.bashrc

# Verify
sim_vehicle.py --help
```

---

### SITL crashes on startup

**Symptoms:**
```
Exception: Failed to start SITL
```

**Solution:**
```bash
# Update MAVProxy
pip3 install --upgrade MAVProxy pymavlink

# Try again
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --map --console
```

---

## MAVROS Connection Issues

### Connection timeout

**Symptoms:**
```
[ERROR] [mavros]: FCU: Connection timeout
```

**Solutions:**

1. **Check SITL is running:**
   - SITL terminal should show "STABILIZE>"
   - Should see "Link 1: Opened"

2. **Check port:**
   ```bash
   netstat -an | grep 14550
   # Should show LISTEN or ESTABLISHED
   ```

3. **Restart in order:**
   ```bash
   pkill -9 -if 'gz sim|gz-sim|sim_vehicle|arducopter|mavproxy|mavros_node|gst-launch|install/simtofly_'
   ./launch/start_sitl.sh      # Wait for "STABILIZE>"
   ./launch/start_mavros.sh    # Then start MAVROS
   ```

---

### No heartbeat from FCU

**Symptoms:**
```
[WARN] [mavros]: No HEARTBEAT messages
```

**Solution:**
```bash
# Check FCU URL in launch file
cat src/simtofly_mavros_sitl/launch/ardupilot_sitl.launch.py
# Should show: fcu_url='udp://:14550@'

# For real hardware, check serial UART connection
ls -l /dev/ttyAMA0
# Should show: crw-rw---- 1 root dialout
```

---

## Real Hardware Issues

### Permission denied: /dev/ttyAMA0

**Symptoms:**
```
Failed to open /dev/ttyAMA0: Permission denied
```

**Solution:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# IMPORTANT: Logout and login (or reboot)
sudo reboot

# Verify after reboot
groups
# Should include: dialout
```

---

### Device not found

**Symptoms:**
```
/dev/ttyAMA0: No such file or directory
```

**Solutions:**

The GPIO UART is a built-in device — it appears once enabled, not when you
plug something in (no USB hot-plug event).

1. **Enable the UART in the boot config (Raspberry Pi example):**
   ```bash
   sudo nano /boot/firmware/config.txt   # older images: /boot/config.txt
   # Ensure these lines exist:
   #   enable_uart=1
   #   dtoverlay=disable-bt
   sudo reboot
   ```

2. **Free the UART from the serial login console:**
   ```bash
   sudo raspi-config   # Interface Options -> Serial Port:
                       #   login shell over serial = NO, hardware enabled = YES
   sudo systemctl disable hciuart
   sudo reboot
   ```

3. **List serial devices to confirm:**
   ```bash
   ls -l /dev/ttyAMA0
   # On a Jetson the UART is /dev/ttyTHS1 instead
   ```

---

### Wrong baud rate

**Symptoms:**
```
Connected but no data from FCU
```

**Solution:**
```bash
# Check Pixhawk 6C parameters (use Mission Planner), TELEM3 = SERIAL4:
SERIAL4_PROTOCOL = 2       # MAVLink2
SERIAL4_BAUD = 921600      # Baud rate

# Match in launch file
cat src/simtofly_mavros_real/launch/pixhawk6c.launch.py
# Should show: /dev/ttyAMA0:921600
```

---

## Mission Script Issues

### Can't arm vehicle

**Symptoms:**
```
Failed to arm
```

**Solutions:**

1. **Check pre-arm errors in SITL console:**
   ```
   PreArm: RC not calibrated
   PreArm: Need 3D Fix
   ```

2. **In SITL, force arm:**
   ```
   STABILIZE> arm throttle force
   ```

3. **Check battery voltage:**
   ```bash
   ros2 topic echo /mavros/battery
   # Voltage should be > 10V
   ```

---

### Mission script hangs

**Symptoms:**
```
Waiting for FCU connection...
[Never connects]
```

**Solution:**
```bash
# Check MAVROS is running
ros2 node list
# Should show: /mavros

# Check connection
ros2 topic echo /mavros/state --once
# connected should be: true
```

---

### Altitude not updating

**Symptoms:**
```
Waiting to reach 10m altitude...
[Stuck at 0.00m]
```

**Solution:**
```bash
# Check QoS settings in script
# MAVROS uses BEST_EFFORT, not RELIABLE

# Verify altitude topic exists
ros2 topic echo /mavros/local_position/pose
# Should show position.z changing
```

---

## Network Issues (Mission Planner)

### Mission Planner can't connect

**Symptoms:**
```
No Heartbeat Packets Received
```

**Solutions:**

1. **Check both devices on same network:**
   ```bash
   # On the companion computer:
   ip addr show wlan0
   
   # On laptop:
   ipconfig  # Windows
   ifconfig  # Linux/Mac
   
   # Both should be 10.73.2.x (or same subnet)
   ```

2. **Test ping:**
   ```bash
   # From companion computer to laptop:
   ping 10.73.2.236
   
   # From laptop to companion computer:
   ping 10.73.2.222
   ```

3. **Check firewall:**
   - Windows: Allow Mission Planner through firewall
   - Linux: `sudo ufw allow 14550/udp`

4. **Verify GCS URL:**
   ```bash
   # In start_mavros_real.sh
   # Should show: gcs_url:=udp://@LAPTOP_IP:14550
   ```

---

## Gate Course Issues (N1–N7)

Every failure in this section looked, from the outside, like a healthy stack. That is
the point: none of them announce themselves.

### The whole stack is silent — no gates, TF errors at time 0.000000

A **stale Gazebo server from the previous run** is still holding the FDM port, so the
readiness wait passes instantly and this run attaches to the *old* world. `/clock`
never reaches ROS, every node sits at time 0, and TF lookups fail forever.

`gz sim` **ignores SIGTERM**, so a plain `pkill -f "gz sim"` does not kill it:

```bash
pkill -KILL -f "gz sim"
ss -lun | grep 9002        # must print NOTHING before you start again
```

### Prearm refuses with "Accels inconsistent" / "Gyros inconsistent"

Two stacks are running at once — almost always a previous run that never died. It has
nothing to do with the IMU. Tear everything down, confirm no `gz sim`, `arducopter` or
`mavros_node` survives, then start one stack.

> Put the kill commands in a **script**, never inline. `pkill -f` matches the argv of
> the shell running it, so any command that merely *mentions* a process name — even
> `grep -n "mission_fsm" file` — can kill its own shell halfway through and leave the
> stack half up.

### The drone flies one or two rings, then hovers until the mission gives up

`gate_planner` logs `dijkstra FAILURE` on every tick. After threading a ring the drone
sits inside that ring's 0.60 m inflation skirt, so its own voxel and all 26 neighbours
read occupied and the search has no edge to leave the start on. Fixed in
`free_around_drone()`; if it reappears, the diagnostic prints the evidence directly:

```
dijkstra FAILURE to gate at [...] | drone (True, 0) P_pre (False, 26)
                                          ^^^^^^^^^  occupied, 0 of 26 free
```

### The mission finishes, then ArduCopter reports `Crash: Disarming: AngErr>30`

The drone landed on a **ring stand** — a 0.35 m base plate directly under each ring,
which is where the course leaves it hovering. Finishing the course must end in `RTL`
(returns to the open launch point), not `LAND`. Aborts still use `LAND` on purpose.

Precision landing looks like the culprit here (`PrecLand: Failsafe Measures` appears
at exactly the right moment, and SITL's `eeprom.bin` may well have `PLND_ENABLED 1`
persisted). It is not — forcing `PLND_ENABLED 0` does not stop the crash. Don't spend
time there.

### Running SITL headless: MAVROS connects but no topic ever publishes

MAVProxy dies on EOF without a tty and takes the vehicle with it, so headless runs
need `--no-mavproxy`. But without MAVProxy nothing requests the data streams, and SITL
leaves every `SR0_*` at 0 — `/mavros/state` says `connected: true` while no data
arrives. Feed the rates in by param file:

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy \
    --add-param-file=rates.parm      # SR0_POSITION 30, SR0_EXTRA1 30, ...
```

30 Hz because `map->base_link` comes from that stream and has to match the 30 Hz depth
image; at MAVProxy's default of 4 Hz every node looking up a transform at an image's
own stamp asks for a time the buffer has not reached.

---

## Still Having Issues?

1. **Check logs:**
   ```bash
   # MAVROS logs
   ros2 run mavros mavros_node --ros-args --log-level debug
   
   # SITL logs
   cat ~/ardupilot/ArduCopter/logs/LASTLOG.TXT
   ```

2. **Ask for help:**
   - [GitHub Issues](https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/issues)
   - [GitHub Discussions](https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware/discussions)

3. **Include in your report:**
   - OS version: `lsb_release -a`
   - ROS2 version: `ros2 --version`
   - Error messages (full output)
   - Steps to reproduce

---

**Author:** phantrungtien  
**GitHub:** https://github.com/phantrungtien

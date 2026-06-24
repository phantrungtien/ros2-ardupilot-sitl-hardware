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
   ./launch/stop_all.sh
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

# For real hardware, check serial connection
ls -l /dev/ttyACM0
# Should show: crw-rw---- 1 root dialout
```

---

## Real Hardware Issues

### Permission denied: /dev/ttyACM0

**Symptoms:**
```
Failed to open /dev/ttyACM0: Permission denied
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
/dev/ttyACM0: No such file or directory
```

**Solutions:**

1. **Check USB connection:**
   ```bash
   # Plug in flight controller
   dmesg | tail -20
   # Should show: ttyACM0: USB ACM device
   ```

2. **List USB devices:**
   ```bash
   ls /dev/tty* | grep -E "ACM|USB"
   # Device might be ttyACM1 or ttyUSB0
   ```

3. **Try different USB cable:**
   - Some cables are power-only
   - Use a data cable

---

### Wrong baud rate

**Symptoms:**
```
Connected but no data from FCU
```

**Solution:**
```bash
# Check Cube Orange parameters (use Mission Planner):
SERIAL0_PROTOCOL = 2       # MAVLink2
SERIAL0_BAUD = 921600      # Baud rate

# Match in launch file
cat src/simtofly_mavros_real/launch/pixhawk6c.launch.py
# Should show: /dev/ttyACM0:921600
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
   # On Raspberry Pi:
   ip addr show wlan0
   
   # On laptop:
   ipconfig  # Windows
   ifconfig  # Linux/Mac
   
   # Both should be 10.73.2.x (or same subnet)
   ```

2. **Test ping:**
   ```bash
   # From RPi to laptop:
   ping 10.73.2.236
   
   # From laptop to RPi:
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

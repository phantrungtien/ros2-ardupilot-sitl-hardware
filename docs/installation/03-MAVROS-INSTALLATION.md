# MAVROS Installation

Install MAVROS (MAVLink extendable communication node for ROS2).
---

## Prerequisites

Complete these first:
- ✅ [01-SYSTEM-SETUP.md](01-SYSTEM-SETUP.md)
- ✅ [02-ROS2-INSTALLATION.md](02-ROS2-INSTALLATION.md)
- ✅ ROS2 Humble installed and sourced

---

## Step 1: Install MAVROS Packages

### Install from binary (recommended)

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
```

**What this installs:**
- `ros-humble-mavros` - Core MAVROS package
- `ros-humble-mavros-extras` - Additional plugins (PX4Flow, etc.)

**This takes:** 2-5 minutes

---

## Step 2: Install GeographicLib Datasets

**⚠️ CRITICAL:** GeographicLib datasets are **required** for GPS coordinate conversions.

### Download and run installation script

```bash
# Download script
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh

# Make executable
chmod +x install_geographiclib_datasets.sh

# Run as sudo
sudo ./install_geographiclib_datasets.sh
```

**What this installs:**
- Geoid dataset (egm96) - Converts GPS altitude formats
- Gravity dataset - Gravity field calculations  
- Magnetic dataset - Magnetic field models

**This takes:** 3-5 minutes (downloads ~100MB)

**Expected output:**
```
Downloading geoid dataset...
...
Installing to /usr/share/GeographicLib/...
...
Done!
```

✅ **Success:** GeographicLib datasets installed

---

## Step 3: Verify Installation

### Check MAVROS packages

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep mavros
```

**Expected output:**
```
mavros
mavros_extras
mavros_msgs
...
```

✅ **Success:** MAVROS packages installed

---

## Step 4: Test MAVROS Connection

### Understanding Connection URLs

MAVROS uses URL format to connect to flight controllers:

**Serial (Real Hardware):**
```
/dev/ttyACM0:921600
```

**UDP (SITL Simulation):**
```
udp://:14550@
```

**TCP:**
```
tcp://127.0.0.1:5760
```

See [MAVROS Connection URL docs](https://github.com/mavlink/mavros/blob/ros2/README.md#connection-url) for all formats.

---

## Next Steps

✅ **MAVROS installed successfully**

### For Simulation Testing:

→ **Continue to:** [04-ARDUPILOT-SITL.md](04-ARDUPILOT-SITL.md)

### For Real Hardware:

→ **See:** [Real Hardware Setup Guide](../guides/REAL_HARDWARE_SETUP.md)

---

## Troubleshooting

### "Failed to find GeographicLib datasets"

**Solution:**
```bash
# Re-run installation script
sudo ./install_geographiclib_datasets.sh

# Verify installation
ls /usr/share/GeographicLib/geoids/
# Should show: egm96-5.pgm
```

### MAVROS node crashes on startup

**Cause:** Missing GeographicLib datasets

**Solution:** Ensure Step 2 completed successfully.

### "Package 'ros-humble-mavros' has no installation candidate"

**Cause:** ROS2 repository not added correctly

**Solution:**
```bash
# Re-add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
```

---

## Additional Resources

- **MAVROS Documentation:** https://github.com/mavlink/mavros
- **MAVLink Protocol:** https://mavlink.io/
- **Connection URLs:** https://github.com/mavlink/mavros/blob/ros2/README.md#connection-url

---

**Repository:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware
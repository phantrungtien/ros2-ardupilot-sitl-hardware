# ArduPilot SITL Installation

Install ArduPilot Software-In-The-Loop (SITL) simulator for safe drone testing.
 
**Detailed Guide:** https://github.com/simtofly/simtofly-guide/blob/main/docs/phase-1-simulation/1.3-ardupilot-sitl.md

---

## What is SITL?

**SITL** = Software In The Loop

A virtual drone running on your computer:
- ✅ No hardware needed
- ✅ Same ArduPilot firmware as real drones
- ✅ Safe to test and crash
- ✅ Perfect for development

---

## Prerequisites

Complete these first:
- ✅ [01-SYSTEM-SETUP.md](01-SYSTEM-SETUP.md)
- ✅ [02-ROS2-INSTALLATION.md](02-ROS2-INSTALLATION.md)
- ✅ [03-MAVROS-INSTALLATION.md](03-MAVROS-INSTALLATION.md)

---

## Step 1: Install ArduPilot Dependencies

```bash
sudo apt install -y \
    git \
    python3-dev \
    python3-opencv \
    python3-wxgtk4.0 \
    python3-pip \
    python3-matplotlib \
    python3-lxml \
    python3-pygame
```

**This takes:** 2-5 minutes

---

## Step 2: Clone ArduPilot Repository

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

**This takes:** 2-5 minutes (depends on internet speed)

**Expected output:**
```
Cloning into 'ardupilot'...
remote: Enumerating objects: ...
...
```

✅ **Success:** ArduPilot repository cloned

---

## Step 3: Checkout Stable Version

```bash
cd ~/ardupilot
git checkout Copter-4.5.7
```

**Why 4.5.7?** Stable release validated with this framework (November 2025).

**Expected output:**
```
HEAD is now at xxxxxxx ArduCopter: release 4.5.7
```

---

## Step 4: Initialize Submodules

```bash
git submodule update --init --recursive
```

**What this does:** Downloads additional required libraries.

**This takes:** 5-10 minutes

**If this fails:** Run this first, then retry:
```bash
git config --global url.https://.insteadOf git://
git submodule update --init --recursive
```

---

## Step 5: Install ArduPilot Tools

```bash
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

**This takes:** 10-20 minutes (installs compilers, libraries, tools)

**Expected output:**
```
---------- Installing prerequisites for SITL ----------
...
Finished installing prerequisites
```

### Reload environment

```bash
. ~/.profile
```

---

## Step 6: Setup Environment Variables

```bash
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## Step 7: First SITL Launch

### Initialize parameters

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```

**Important:** The `-w` flag wipes parameters (only use first time).

**Wait for:** `Received XXX parameters` message

**Then:** Press `Ctrl+C` to exit

---

## Step 8: Verify Installation

### Launch SITL

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter
```

**Expected output:**
```
Starting SITL...
...
Init ArduCopter V4.5.7
...
STABILIZE>
```

✅ **Success:** SITL running, shows `STABILIZE>` prompt

### Test basic commands

At the `STABILIZE>` prompt:

```
mode GUIDED
arm throttle
disarm
```

**Expected:** Mode changes, vehicle arms/disarms

**To exit:** Press `Ctrl+C`

---

## Troubleshooting

### "sim_vehicle.py: command not found"

**Solution:**
```bash
echo 'export PATH=$PATH:$HOME/ardupilot/Tools/autotest' >> ~/.bashrc
source ~/.bashrc
```

### Submodule update fails

**Solution:**
```bash
git config --global url.https://.insteadOf git://
git submodule update --init --recursive
```

### Build fails

**Solution:**
```bash
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### "No module named 'pymavlink'"

**Solution:**
```bash
pip3 install --user pymavlink MAVProxy
```

---

## Next Steps

✅ **ArduPilot SITL installed successfully**

### Ready to Use This Framework:

```bash
# Clone this repository
git clone https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware.git
cd ros2-ardupilot-sitl-hardware

# Build packages
colcon build
source install/setup.bash

# Test in simulation
./launch/start_sitl.sh      # Terminal 1
./launch/start_mavros.sh    # Terminal 2
source install/setup.bash && python3 scripts/missions/mission_simple.py  # Terminal 3
```

→ **Continue to:** [SITL Setup Guide](../guides/SITL_SETUP.md)

---

## Additional Resources

- **Detailed SITL Tutorial:** https://github.com/simtofly/simtofly-guide/
- **ArduPilot Wiki:** https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- **MAVProxy Docs:** https://ardupilot.org/mavproxy/

---

**Repository:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware

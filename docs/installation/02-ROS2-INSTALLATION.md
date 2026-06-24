# ROS2 Humble Installation

Install ROS2 Humble Hawksbill on Ubuntu 22.04.

---

## Prerequisites

Complete **[01-SYSTEM-SETUP.md](01-SYSTEM-SETUP.md)** first:
- ✅ Locale set to UTF-8
- ✅ System fully updated
- ✅ Essential tools installed

---

## Step 1: Setup ROS2 Sources

### Enable Ubuntu Universe repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

### Add ROS2 GPG key

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Add repository to sources list

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

---

## Step 2: Install ROS2 Packages

### Update apt cache

```bash
sudo apt update
```

### Install ROS2 Desktop (Recommended)

```bash
sudo apt install ros-humble-desktop -y
```

**Includes:** ROS, RViz, demos, tutorials, GUI tools

**Alternative (Minimal):**
```bash
# ROS-Base (no GUI tools)
sudo apt install ros-humble-ros-base -y
```

**This takes:** 5-10 minutes

---

## Step 3: Install Development Tools

```bash
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

---

## Step 4: Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

**Expected output:**
```
Wrote /etc/ros/rosdep/sources.list.d/20-default.list
...
reading in sources list data from /etc/ros/rosdep/sources.list.d
...
```

✅ **Success:** rosdep initialized

---

## Step 5: Setup Environment

### Source ROS2 setup file

```bash
source /opt/ros/humble/setup.bash
```

### Add to .bashrc (permanent)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 6: Verify Installation

### Check ROS2 version

```bash
ros2 --version
```

**Expected output:**
```
ros2 cli version 0.10.x
```

### Test ROS2 nodes

**Terminal 1:**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2:**
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

**Expected:** Listener receives messages from talker.

✅ **Success:** ROS2 Humble installed correctly!

---

## Troubleshooting

### "ros2: command not found"

**Solution:**
```bash
source /opt/ros/humble/setup.bash
```

### rosdep init fails with "already initialized"

**Solution:**
```bash
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init
rosdep update
```

### Package not found errors

**Solution:**
```bash
sudo apt update
sudo apt install ros-humble-[package-name]
```

---

## Next Steps

✅ **ROS2 Humble installed successfully**

→ **Continue to:** [03-MAVROS-INSTALLATION.md](03-MAVROS-INSTALLATION.md)

---
**Repository:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware  

**Official Docs:** https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

**ROS2 Docs:** https://docs.ros.org/en/humble/

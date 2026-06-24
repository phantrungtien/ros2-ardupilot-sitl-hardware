# System Setup

Prepare your Ubuntu system for ROS2 and ArduPilot development.

---

## Prerequisites

- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **RAM:** Minimum 4GB (8GB recommended)
- **Disk:** 10GB free space
- **Architecture:** amd64 or arm64

---

## Step 1: Set Locale

Ensure your system supports UTF-8:

```bash
# Check current locale
locale

# Install locales package
sudo apt update && sudo apt install locales

# Generate UTF-8 locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Export for current session
export LANG=en_US.UTF-8

# Verify
locale
```

**Expected output:**
```
LANG=en_US.UTF-8
LC_ALL=en_US.UTF-8
...
```

✅ **Success:** UTF-8 locale configured

---

## Step 2: Update System

### Update package lists

```bash
sudo apt update
```

### Upgrade packages (CRITICAL)

```bash
sudo apt upgrade -y
```

**⚠️ Important:** This step is **mandatory** before installing ROS2.

**Why?** Early Ubuntu 22.04 updates can cause critical system package removal if ROS2 is installed without upgrading first. See [ros2/ros2#1272](https://github.com/ros2/ros2/issues/1272).

**This takes:** 5-15 minutes depending on pending updates.

---

## Step 3: Install Essential Tools

```bash
sudo apt install -y \
    software-properties-common \
    build-essential \
    git \
    curl \
    wget \
    python3-pip
```

**What these are:**
- `software-properties-common` - Manage software repositories
- `build-essential` - C/C++ compiler and build tools
- `git` - Version control
- `curl`, `wget` - Download tools
- `python3-pip` - Python package manager

---

## Step 4: Verify Installation

```bash
# Check tools
gcc --version
git --version
python3 --version
pip3 --version
```

**Expected output:**
```
gcc (Ubuntu 11.x.x) ...
git version 2.34.x
Python 3.10.x
pip 22.x.x
```

✅ **Success:** All tools installed

---

## Next Steps

✅ **System ready for ROS2 installation**

→ **Continue to:** [02-ROS2-INSTALLATION.md](02-ROS2-INSTALLATION.md)

---

**Repository:** https://github.com/phantrungtien/ros2-ardupilot-sitl-hardware

# Mission Planner Connection Setup

**Author:** phantrungtien
**GitHub:** https://github.com/phantrungtien

Guide to connect Mission Planner on your laptop to MAVROS running on the companion computer.

---

## Network Setup

> **Note:** The IP addresses below are examples from our test setup. Replace them with your own network IPs. To find your IP, run `ip addr show wlan0` on the companion computer or `ipconfig` on Windows.

**Example Network Configuration:**
- **Companion computer IP:** 10.73.2.222 *(replace with your companion computer IP)*
- **Laptop IP:** 10.73.2.236 *(replace with your laptop IP)*
- **Network:** Same WiFi network
- **Protocol:** UDP on port 14550

---

## STEP 1: Start MAVROS with GCS Forwarding

On the **companion computer**, run:

```bash
cd ~/ros2-ardupilot-sitl-hardware
./launch/start_mavros_real.sh
```

**When prompted:**
1. Confirm safety checks: `y`
2. Send telemetry to Mission Planner? `y`
3. Enter laptop IP: `10.73.2.236` (or press Enter for default)

**Expected output:**
```
✓ GCS URL: Sending telemetry to 10.73.2.236:14550

Mission Planner Setup:
  1. Open Mission Planner on laptop (10.73.2.236)
  2. Click 'Connect' dropdown → UDP
  3. Port: 14550
  4. Click 'Connect'
```

---

## STEP 2: Configure Mission Planner (On Laptop)

### Method 1: UDP Listen Mode (Recommended)

1. **Open Mission Planner** on your laptop (10.73.2.236)

2. **Top-right corner:**
   - Click the dropdown (says "COM1" or similar)
   - Select **"UDP"**

3. **Port Configuration:**
   - A dialog will appear asking for port
   - Enter: **14550**
   - Click **OK**

4. **Click "Connect" button**

5. **Wait for connection:**
   - Status should show: "Getting Params..."
   - Then: "Connected"
   - HUD should start showing data

---

### Method 2: Manual UDP Configuration

If dropdown doesn't work:

1. In Mission Planner, go to: **INITIAL SETUP → Optional Hardware → Antenna Tracker**
2. Connection Type: **UDP Client**
3. Remote Host: **10.73.2.222** (companion computer IP)
4. Remote Port: **14550**
5. Click **Connect**

---

## STEP 3: Verify Connection

Once connected, you should see:

✅ **HUD (Heads-Up Display):**
- Artificial horizon updating
- Altitude, speed displayed
- GPS coordinates

✅ **Flight Data Tab:**
- Battery voltage
- GPS status
- Flight mode (STABILIZE, GUIDED, etc.)
- Attitude (roll, pitch, yaw)

✅ **Messages Tab:**
- Real-time messages from ArduPilot
- No "No Heartbeat" errors

---

## Troubleshooting

### Connection Failed / Timeout

**Problem:** Mission Planner can't connect

**Solutions:**

1. **Check both devices on same network:**
   ```bash
   # On the companion computer:
   ip addr show wlan0 | grep "inet "

   # On Laptop (Windows Command Prompt):
   ipconfig
   ```
   Both should be on `10.73.2.x` network

2. **Verify MAVROS is running on the companion computer:**
   ```bash
   # On the companion computer:
   ros2 topic list | grep mavros
   ```
   Should show many topics

3. **Check firewall on laptop:**
   - Windows Defender Firewall might block UDP 14550
   - Allow Mission Planner through firewall

4. **Ping test:**
   ```bash
   # On the companion computer, ping laptop:
   ping 10.73.2.236

   # On Laptop (Command Prompt), ping the companion computer:
   ping 10.73.2.222
   ```

---

### "No Heartbeat" Error

**Problem:** Connected but no data

**Solution:**
- Restart MAVROS on the companion computer
- Check the Pixhawk 6C is powered and connected
- Verify TELEM3 UART wiring (Pixhawk 6C → companion computer)

---

### Wrong Parameters Shown

**Problem:** Mission Planner shows different vehicle

**Solution:**
- Disconnect Mission Planner
- Restart MAVROS on the companion computer
- Reconnect Mission Planner

---

## Network Configuration Reference

### Current Setup:

```
Laptop (Mission Planner)          Companion computer (MAVROS)
10.73.2.236                       10.73.2.222
     |                                  |
     └─────── UDP Port 14550 ───────────┘
                    ↑
                    |
              MAVLink Telemetry
```

### Data Flow:

```
Pixhawk 6C (TELEM3 UART)
    ↓ MAVLink
Companion computer (MAVROS)
    ↓ UDP:14550
Laptop (Mission Planner)
```

---

## Alternative: Direct Manual Command

If you want to run MAVROS manually with GCS URL:

```bash
source /opt/ros/humble/setup.bash

ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="/dev/ttyAMA0:921600" \
    -p gcs_url:="udp://@10.73.2.236:14550" \
    -p target_system_id:=1 \
    -p target_component_id:=1
```

**Change `10.73.2.236` to your laptop's current IP if it changes.**

---

## Using QGroundControl Instead

If using QGroundControl instead of Mission Planner:

1. **Open QGroundControl**
2. **Application Settings → Comm Links**
3. **Add new connection:**
   - Name: `ROS2 MAVROS`
   - Type: `UDP`
   - Listening Port: `14550`
4. **Connect**

QGC will automatically listen on UDP 14550.

---

## Changing Laptop IP

If your laptop IP changes, update the script:

**Option 1: Script will prompt you**
```bash
./launch/start_mavros_real.sh
# Enter new IP when prompted
```

**Option 2: Edit default in script**
```bash
nano ~/ros2-ardupilot-sitl-hardware/launch/start_mavros_real.sh
# Change line: LAPTOP_IP=${LAPTOP_IP:-10.73.2.236}
# To new IP:   LAPTOP_IP=${LAPTOP_IP:-10.73.2.XXX}
```

---

## Static IP for the Companion Computer (Future Setup)

To make the companion computer always have the same IP (10.73.2.222):

**Edit netplan config:**
```bash
sudo nano /etc/netplan/99-custom.yaml
```

**Add:**
```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: no
      addresses:
        - 10.73.2.222/24
      gateway4: 10.73.2.80
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
      access-points:
        "YourWiFiName":
          password: "YourWiFiPassword"
```

**Apply:**
```bash
sudo netplan apply
```

---

## Multiple Ground Stations

You can send telemetry to multiple laptops:

```bash
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:="/dev/ttyAMA0:921600" \
    -p gcs_url:="udp://@10.73.2.236:14550,udp://@10.73.2.240:14550" \
    -p target_system_id:=1 \
    -p target_component_id:=1
```

Each laptop can run Mission Planner/QGC on port 14550.

---

## Summary Checklist

**Before connecting Mission Planner:**

- [ ] Pixhawk 6C powered and wired to the companion computer (TELEM3 UART)
- [ ] Companion computer and Laptop on same WiFi network
- [ ] MAVROS running on the companion computer with GCS URL set
- [ ] Mission Planner configured for UDP port 14550
- [ ] No firewall blocking UDP 14550

**Expected result:**
- ✅ Mission Planner shows "Connected"
- ✅ HUD displays real-time data
- ✅ Can see flight mode, battery, GPS
- ✅ Can change flight modes from Mission Planner
- ✅ Can monitor telemetry in real-time

---

**Happy Flying!** 🚁

**Author:** phantrungtien
**GitHub:** https://github.com/phantrungtien

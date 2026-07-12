#!/usr/bin/env python3
"""One-shot launch for the CAMERA gates mission -- everything in one file.

Brings up, each in its OWN gnome-terminal window (so logs stay separate) and
gated by READINESS not timers:

  1. Gazebo      - gz sim with drone_vision/worlds/gates_course.sdf: the iris
                   carrying a RealSense D435i on its nose, and one orange ring
  2. SITL        - ArduCopter bound to Gazebo (waits for the FDM port 9002)
  3. DepthBridge - ros_gz_bridge: the D435i depth image -> ROS, which is what
                   gate_detect ranges the ring with
  4. MAVROS      - waits for SITL's master port (TCP 5760)
  5. Controller  - mission_gates (waits for /mavros/state connected)
  6. GateVision  - gate_detect WITH its GUI window (--gui): the live colour feed
                   with the detected ring/ellipse overlay. It publishes the
                   /gate/obs that (5) flies by.

The two camera streams reach us by different routes, which is why (3) exists at
all: COLOUR still arrives as H264/RTP on UDP 5600 (the colour sensor carries
ardupilot_gazebo's GstCameraPlugin), so gate_detect reads it with gst-launch and
needs no bridge. DEPTH is a plain Gazebo topic with no such plugin, so it has to
be bridged into ROS.

This is the gates-specific sibling of sitl_mission.launch.py. Both now fly the gates:
`sitl_mission.launch.py mission:=mission_gates` picks gates_course.sdf and the depth
bridge on its own. The only thing you get HERE and not there is the vision GUI --
gate_detect runs with --gui, so you can watch the ellipse it fits on the ring.

Run it from a DESKTOP terminal (gnome-terminal needs a real session; it will not
spawn from a headless/non-interactive shell).

Usage:
    ros2 launch simtofly_mavros_sitl gates_mission.launch.py

Requirements (already on this machine): ArduPilot at ~/ardupilot, the gz_ws at
~/gz_ws with ardupilot_gazebo built and drone_vision checked out, ros_gz_bridge,
gz + gnome-terminal + ss + timeout, and the workspace built (colcon build).
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess

# bash -c does not read ~/.bashrc, so every window sets up its own env.
ROS_SETUP = (
    'source /opt/ros/humble/setup.bash\n'
    'source $HOME/ros2-ardupilot-sitl-hardware/install/setup.bash\n'
)

# ---- Readiness gates (bounded wait-loops) --------------------------------
WAIT_GAZEBO = (
    'echo "[SITL] Doi Gazebo (plugin FDM cong 9002)..."\n'
    'for i in $(seq 1 120); do\n'
    '  ss -lun 2>/dev/null | grep -qE ":9002\\b" && break\n'
    '  ss -ltn 2>/dev/null | grep -qE ":9002\\b" && break\n'
    '  sleep 1\n'
    'done\n'
)
WAIT_SITL = (
    'echo "[MAVROS] Doi SITL (master TCP 5760)..."\n'
    'for i in $(seq 1 120); do\n'
    '  ss -ltn 2>/dev/null | grep -qE ":5760\\b" && break\n'
    '  sleep 1\n'
    'done\n'
)
WAIT_MAVROS = (
    'echo "[NODE] Doi MAVROS connected (/mavros/state)..."\n'
    'for i in $(seq 1 100); do\n'
    '  timeout 3 ros2 topic echo --once /mavros/state 2>/dev/null '
    '| grep -qE "^connected: true" && break\n'
    '  sleep 1\n'
    'done\n'
)

# ---- Step commands -------------------------------------------------------

# The models live in TWO repos: gates_course.sdf and the D435i/ring/iris models in
# drone_vision, iris_with_standoffs and ArduPilotPlugin.so in ardupilot_gazebo.
# gz_ws/setup.bash is the single place that knows how to put both on the path --
# so source it rather than re-deriving GZ_SIM_RESOURCE_PATH here and drifting.
GAZEBO_CMD = (
    'export GZ_VERSION=harmonic\n'          # setup.bash does not set this one
    'source $HOME/gz_ws/setup.bash\n'
    'gz sim -v4 -r $HOME/gz_ws/src/drone_vision/worlds/gates_course.sdf'
)
SITL_CMD = WAIT_GAZEBO + (
    'cd $HOME/ardupilot/ArduCopter\n'
    '$HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris '
    '--model JSON --map --console --out=udp:127.0.0.1:14550'
)
# Depth only. camera_info is NOT bridged: gate_detect derives the intrinsics from
# the same <horizontal_fov> the SDF declares, so there is one source of truth and
# nothing to keep in sync at runtime.
BRIDGE_CMD = (
    'source /opt/ros/humble/setup.bash\n'
    + WAIT_GAZEBO +
    'ros2 run ros_gz_bridge parameter_bridge '
    "'/d435i/depth/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image'"
)
# apm.launch with the `param` plugin denied -- it aborts MAVROS on duplicate parameter
# replies, which a half-speed sim produces at random (std::future_error: Promise already
# satisfied). apm.launch hardcodes its plugin list, so we ship our own launch instead.
# See mavros_apm.launch.py.
MAVROS_CMD = (
    ROS_SETUP
    + WAIT_SITL +
    'ros2 launch simtofly_mavros_sitl mavros_apm.launch.py fcu_url:=udp://:14550@'
)
MISSION_CMD = ROS_SETUP + WAIT_MAVROS + 'ros2 run simtofly_mavros_sitl mission_gates'
# GATE_GUI=1 (and --gui) -> gate_detect opens its OpenCV overlay window.
VISION_CMD = (ROS_SETUP + WAIT_MAVROS
              + 'export GATE_GUI=1\n'
              + 'ros2 run simtofly_vision gate_detect --gui')


def gnome_term(title, body):
    """Run `body` (a shell script string) in its own gnome-terminal window,
    kept open afterwards (exec bash) so logs stay readable on a crash."""
    return ExecuteProcess(
        prefix='gnome-terminal --title={} --'.format(title),
        cmd=['bash', '-c', body + '\nexec bash'],
        output='screen',
    )


def generate_launch_description():
    # All windows open at once; ordering comes from the wait-loops, not timers.
    return LaunchDescription([
        gnome_term('Gazebo', GAZEBO_CMD),
        gnome_term('SITL', SITL_CMD),
        gnome_term('DepthBridge', BRIDGE_CMD),
        gnome_term('MAVROS', MAVROS_CMD),
        gnome_term('Mission', MISSION_CMD),
        gnome_term('GateVision', VISION_CMD),
    ])

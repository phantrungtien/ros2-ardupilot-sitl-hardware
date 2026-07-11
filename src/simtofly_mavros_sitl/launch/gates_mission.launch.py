#!/usr/bin/env python3
"""One-shot launch for the CAMERA gates mission -- everything in one file.

Brings up, each in its OWN gnome-terminal window (so logs stay separate) and
gated by READINESS not timers:

  1. Gazebo      - gz sim with worlds/gates_course.sdf (the ring + gimbal cam)
  2. SITL        - ArduCopter bound to Gazebo (waits for the FDM port 9002)
  3. MAVROS      - waits for SITL's master port (TCP 5760)
  4. Controller  - mission_gates (waits for /mavros/state connected)
  5. GateVision  - gate_detect WITH its GUI window (--gui): the live camera
                   feed with the detected ring/ellipse overlay. It also does
                   the detection and publishes /gate/obs that (4) flies by.

This is the gates-specific sibling of sitl_mission.launch.py, with the vision
GUI turned on. Run it from a DESKTOP terminal (gnome-terminal needs a real
session; it will not spawn from a headless/non-interactive shell).

Usage:
    ros2 launch simtofly_mavros_sitl gates_mission.launch.py

Requirements (already on this machine): ArduPilot at ~/ardupilot,
ardupilot_gazebo built at ~/gz_ws/src/ardupilot_gazebo/build, gz + gnome-
terminal + ss + timeout, and the workspace built (colcon build).
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
GAZEBO_CMD = (
    'export GZ_VERSION=harmonic\n'
    'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build\n'
    'export GZ_SIM_RESOURCE_PATH='
    '$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds\n'
    'cd $HOME/gz_ws/src/ardupilot_gazebo\n'
    'gz sim -v4 -r worlds/gates_course.sdf'
)
SITL_CMD = WAIT_GAZEBO + (
    'cd $HOME/ardupilot/ArduCopter\n'
    '$HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris '
    '--model JSON --map --console --out=udp:127.0.0.1:14550'
)
MAVROS_CMD = (
    'source /opt/ros/humble/setup.bash\n'
    + WAIT_SITL +
    'ros2 launch mavros apm.launch fcu_url:=udp://:14550@'
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
        gnome_term('MAVROS', MAVROS_CMD),
        gnome_term('Mission', MISSION_CMD),
        gnome_term('GateVision', VISION_CMD),
    ])

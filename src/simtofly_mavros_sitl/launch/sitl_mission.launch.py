#!/usr/bin/env python3
"""
All-in-one launch for ArduPilot SITL + Gazebo, gated by READINESS (not timers).

Each step runs in its OWN gnome-terminal window (via prefix='gnome-terminal --')
so the logs of Gazebo, SITL, MAVROS and the mission do NOT mix together.

Instead of fixed delays, every window WAITS for its dependency to be ready
before starting its own program:

  1. Gazebo  - starts immediately (gz sim loads the world + ArduPilot plugin)
  2. SITL    - waits until the Gazebo plugin's FDM port (UDP 9002) is bound,
               then runs sim_vehicle.py -f gazebo-iris
  3. MAVROS  - waits until SITL's master port (TCP 5760) is listening,
               then runs ros2 launch mavros apm.launch
  4. Mission - waits until /mavros/state reports `connected: true`,
               then runs ros2 run (executable from the `mission` argument)

All four windows open at once; the ordering comes from the wait-loops, not from
timing. Each wait loop is BOUNDED (timeout) so a window never hangs forever
silently -- on timeout it proceeds anyway and its program fails visibly in its
own window (which is easy to read because the logs are not mixed).

Usage:
  ros2 launch simtofly_mavros_sitl sitl_mission.launch.py
  ros2 launch simtofly_mavros_sitl sitl_mission.launch.py mission:=trackinghuman

Notes:
  - Each window stays open after its process exits (`exec bash`) so logs remain
    readable on a crash. (Pressing Ctrl+C inside a window still closes it.)
  - gnome-terminal detaches: Ctrl+C on `ros2 launch` does NOT close the windows.
  - bash -c does not source ~/.bashrc, so each window sets env / sources ROS itself.

Requirements (already present on this machine):
  - ArduPilot at ~/ardupilot, ardupilot_gazebo built at ~/gz_ws/src/ardupilot_gazebo/build
  - Gazebo (gz), gnome-terminal, ss (iproute2), timeout (coreutils)
  - Workspace built: colcon build (entry_points registered in setup.py)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# Source ROS + this workspace overlay (bash -c skips ~/.bashrc).
ROS_SETUP = (
    'source /opt/ros/humble/setup.bash\n'
    'source $HOME/ros2-ardupilot-sitl-hardware/install/setup.bash\n'
)

# ---- Readiness gates (bounded wait-loops) --------------------------------

# Gazebo ready := the ArduPilot plugin has bound its FDM port (UDP 9002, taken
# from the iris model.sdf <fdm_port_in>). Cap ~120s, then proceed anyway.
WAIT_GAZEBO = (
    'echo "[SITL] Doi Gazebo san sang (plugin FDM cong 9002)..."\n'
    'for i in $(seq 1 120); do\n'
    '  ss -lun 2>/dev/null | grep -qE ":9002\\b" && break\n'
    '  ss -ltn 2>/dev/null | grep -qE ":9002\\b" && break\n'
    '  sleep 1\n'
    'done\n'
    'echo "[SITL] Gazebo san sang -> khoi dong SITL"\n'
)

# SITL ready := the master MAVLink port (TCP 5760) is listening.
WAIT_SITL = (
    'echo "[MAVROS] Doi SITL san sang (master TCP 5760)..."\n'
    'for i in $(seq 1 120); do\n'
    '  ss -ltn 2>/dev/null | grep -qE ":5760\\b" && break\n'
    '  sleep 1\n'
    'done\n'
    'echo "[MAVROS] SITL san sang -> khoi dong MAVROS"\n'
)

# MAVROS ready := /mavros/state publishes connected: true. timeout per poll so a
# missing topic (MAVROS not up yet) cannot block the loop.
WAIT_MAVROS = (
    'echo "[MISSION] Doi MAVROS connected (/mavros/state)..."\n'
    'for i in $(seq 1 100); do\n'
    '  timeout 3 ros2 topic echo --once /mavros/state 2>/dev/null '
    '| grep -qE "^connected: true" && break\n'
    '  sleep 1\n'
    'done\n'
    'echo "[MISSION] MAVROS connected -> chay mission"\n'
)

# ---- Step commands -------------------------------------------------------

# 1) Gazebo - cd ~/gz_ws/src/ardupilot_gazebo && gz sim -v4 -r worlds/iris_runway.sdf
GAZEBO_CMD = (
    'export GZ_VERSION=harmonic\n'
    'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build\n'
    'export GZ_SIM_RESOURCE_PATH='
    '$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds\n'
    'cd $HOME/gz_ws/src/ardupilot_gazebo\n'
    'gz sim -v4 -r worlds/iris_runway.sdf'
)

# 2) ArduPilot SITL bound to Gazebo (absolute sim_vehicle.py path -> no PATH dep).
#    Waits for Gazebo's FDM port first.
SITL_CMD = WAIT_GAZEBO + (
    'cd $HOME/ardupilot/ArduCopter\n'
    '$HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris '
    '--model JSON --map --console --out=udp:127.0.0.1:14550'
)

# 3) MAVROS - official ArduPilot launch (apm_pluginlists.yaml + apm_config.yaml).
#    Waits for SITL's master port first. apm.launch lives in /opt/ros (no overlay).
MAVROS_CMD = (
    'source /opt/ros/humble/setup.bash\n'
    + WAIT_SITL +
    'ros2 launch mavros apm.launch fcu_url:=udp://:14550@'
)


def gnome_term(title, *parts):
    """ExecuteProcess that runs `parts` (shell command pieces) in its own
    gnome-terminal window, kept open afterwards (`exec bash`) for readable logs.

    `parts` may include launch substitutions (e.g. the mission name)."""
    return ExecuteProcess(
        prefix=f'gnome-terminal --title={title} --',
        cmd=['bash', '-c', list(parts) + ['\nexec bash']],
        output='screen',
    )


def generate_launch_description():
    mission = LaunchConfiguration('mission')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission',
            default_value='mission_simple',
            description='Mission executable: mission_simple | trackinghuman',
        ),

        # All four windows open at once. Ordering is enforced by the wait-loops
        # inside each window, not by timers.
        gnome_term('Gazebo', GAZEBO_CMD),
        gnome_term('SITL', SITL_CMD),
        gnome_term('MAVROS', MAVROS_CMD),
        gnome_term(
            'Mission',
            ROS_SETUP + WAIT_MAVROS + 'ros2 run simtofly_mavros_sitl ', mission,
        ),
    ])

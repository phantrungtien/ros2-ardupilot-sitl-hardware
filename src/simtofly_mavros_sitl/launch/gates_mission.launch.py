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
  7. CameraTF    - the static base_link -> d435i_depth_optical_frame transform.
                   Nothing else publishes it: MAVROS gives map->base_link and gz
                   stamps the depth image d435i_depth_optical_frame, but the link
                   BETWEEN them is ours to state. Without it gate_tracker and
                   gate_planner cannot reach `map` and silently drop every frame,
                   so the whole planner stack below goes dark. See CAMERA_TF.
  8. GateTracker - gate_tracker (N2): /gate/detections -> /gate/map
  9. GatePlanner - gate_planner (N3+N4): depth -> occupancy grid, and Dijkstra over
                   it -> /planner/path (latched). One node, because the grid never
                   needed to leave the process -- see the class docstring.

(8)-(9) are the Dijkstra planning stack. They run ALONGSIDE mission_gates (5),
not instead of it: (5) still flies the drone off its own detections, while (9)
publishes the path it WOULD fly. Nothing consumes /planner/path yet -- N5 does not
exist -- so watch it in RViz. When N5/N6 land they replace (5), not these.

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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
# --streamrate 30: MAVProxy defaults to FOUR Hz, and that is what sets how often
# MAVROS gets LOCAL_POSITION_NED -- so map->base_link was published at 4.1 Hz while
# depth arrived at 30.4 Hz. Every node that looks a transform up at an image's own
# stamp then asks for a time the buffer has not reached yet: measured lag p50 0.12 s,
# p99 0.30 s, and gate_tracker/gate_planner logging "extrapolation into the future"
# on ~1 frame in 20. Raising the rate fixes the cause rather than the symptom -- and
# a 4 Hz pose is too stale to build a map from once the drone is actually moving.
SITL_CMD = WAIT_GAZEBO + (
    'cd $HOME/ardupilot/ArduCopter\n'
    '$HOME/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris '
    '--model JSON --map --console --out=udp:127.0.0.1:14550 '
    '--mavproxy-args="--streamrate=30"'
)
# Full D435i bridge (colour + depth + both camera_info + cloud + imu), driven by
# config/d435i_bridge.yaml so the topic list has one source of truth. The new
# vision/planner stack reads bridged ROS topics; the legacy gate_detect still
# pulls colour over its own gst/UDP path and is unaffected by the extra topics.
BRIDGE_CMD = (
    ROS_SETUP
    + WAIT_GAZEBO +
    'CFG=$(ros2 pkg prefix simtofly_mavros_sitl)'
    '/share/simtofly_mavros_sitl/config/d435i_bridge.yaml\n'
    'ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$CFG'
)
# apm.launch with the `param` plugin denied -- it aborts MAVROS on duplicate parameter
# replies, which a half-speed sim produces at random (std::future_error: Promise already
# satisfied). apm.launch hardcodes its plugin list, so we ship our own launch instead.
# See mavros_apm.launch.py.
MAVROS_CMD = (
    ROS_SETUP
    + WAIT_SITL +
    'ros2 launch simtofly_mavros_sitl mavros_apm.launch.py fcu_url:=udp://:14550@ '
    'use_sim_time:=true'
)
# Every ROS node in this launch runs on GAZEBO's clock, not the wall clock. The
# camera images are stamped with sim time at the source, so unless MAVROS stamps
# its TF the same way, no node can look a transform up at an image's own stamp --
# see the /clock entry in d435i_bridge.yaml for what that failure looks like.
SIM_TIME = ' --ros-args -p use_sim_time:=true'

# Which controller flies the drone. Default hold_and_watch, NOT mission_gates:
#   hold_and_watch - takes off and holds still facing gate_1. What you want while
#                    verifying N1..N4, because a stationary drone keeps the ring in
#                    frame 100% of the time instead of 1.3% (measured), which is
#                    the difference between gate_tracker CONFIRMing and starving.
#   mission_gates  - the legacy vision FSM. Currently DOES NOT WORK: it subscribes
#                    /gate/obs, which nothing has published since gate_detect was
#                    rewritten to emit /gate/detections. It takes off and yaw-sweeps
#                    forever. Kept selectable so the old behaviour is one arg away.
CONTROLLER = LaunchConfiguration('controller')
# Flags for the controller itself, e.g. controller_args:=--probe-accel. Goes
# through the launch rather than a hand-run terminal on purpose: only ONE node may
# stream setpoints, so running a second controller alongside this one fights it.
CONTROLLER_ARGS = LaunchConfiguration('controller_args')
MISSION_CMD = [ROS_SETUP + WAIT_MAVROS + 'ros2 run simtofly_mavros_sitl ',
               CONTROLLER, ' ', CONTROLLER_ARGS, SIM_TIME]
# GATE_GUI=1 (and --gui) -> gate_detect opens its OpenCV overlay window.
VISION_CMD = (ROS_SETUP + WAIT_MAVROS
              + 'export GATE_GUI=1\n'
              + 'ros2 run simtofly_vision gate_detect --gui' + SIM_TIME)

# The camera extrinsic, and the ONLY place it is stated. Translation is straight
# out of drone_vision/models/iris_with_d435i/model.sdf: the d435i include sits at
# (0.116, 0.0175, 0) on base_link, and the depth sensor sits at (0,0,0) inside it.
# The rotation is the standard ROS optical convention -- base_link is FLU (x fwd,
# y left, z up), an optical frame is x-right, y-down, z-forward -- which is exactly
# rpy (-90, 0, -90) deg. That matches what N1 and N3 PRODUCE: both back-project the
# depth image themselves into true optical axes rather than trusting gz's cloud,
# whose data is x-forward despite the optical frame_id.
CAMERA_TF_ARGS = [
    '--x', '0.116', '--y', '0.0175', '--z', '0',
    '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
    '--frame-id', 'base_link',
    '--child-frame-id', 'd435i_depth_optical_frame',
]
# The COLOUR optical frame, which is a different frame and needs saying so.
# Everything above N1 works off depth, so this was never needed until N7 -- and
# N7 fails SILENTLY without it: colour images are stamped
# d435i_color_optical_frame, tf2 has no such frame, and the overlay simply never
# draws while every node around it looks healthy.
# y = 0.0175 + 0.015: model.sdf puts d435i_color_frame one 15 mm colour/depth
# baseline left of camera_link, and camera_link 0.0175 left of base_link. Same
# optical rotation as depth.
COLOR_TF_ARGS = [
    '--x', '0.116', '--y', '0.0325', '--z', '0',
    '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
    '--frame-id', 'base_link',
    '--child-frame-id', 'd435i_color_optical_frame',
]
TRACKER_CMD = (ROS_SETUP + WAIT_MAVROS
               + 'ros2 run simtofly_vision gate_tracker' + SIM_TIME)
# gate_planner IS N3+N4: it builds the voxel grid from depth and plans on it in one
# process. There is no separate voxel_mapper node -- shipping the grid between two
# nodes as an RViz marker list dropped nearly all of it, and the planner ended up
# planning against a blank map. It still publishes /mapping/voxels for RViz.
PLANNER_CMD = (ROS_SETUP + WAIT_MAVROS
               + 'ros2 run simtofly_planner gate_planner' + SIM_TIME)
# N6. OPT-IN (fsm:=true), because it ENDS the mission: with a controller that
# never threads a ring -- hold_and_watch just hovers -- every gate eventually
# hits GATE_TIMEOUT, N6 calls the course finished and lands the drone out from
# under whatever you were verifying. Only turn it on with a controller that
# actually flies the gates.
FSM_CMD = (ROS_SETUP + WAIT_MAVROS
           + 'ros2 run simtofly_mavros_sitl mission_fsm' + SIM_TIME)
# N7. Draws the plan onto the colour image and republishes it; watch with
# `ros2 run rqt_image_view rqt_image_view /plan_overlay/image`. Always on: it
# only ever reads and publishes its own topic, so it cannot disturb a flight.
OVERLAY_CMD = (ROS_SETUP + WAIT_MAVROS
               + 'ros2 run simtofly_planner plan_overlay' + SIM_TIME)


# Printed the moment the window's process exits. Without it the window just sits
# there showing its last line of output, which is indistinguishable from a node
# that is alive and quiet -- a dead stack can be read as a running one for minutes.
# `exec bash` keeps the window open on purpose (the logs are the point), so the
# banner is what tells the two states apart.
DIED_BANNER = (
    '\nrc=$?\n'
    'echo\n'
    'echo "============================================================"\n'
    'echo "  *** {title} DA THOAT (exit code $rc) ***"\n'
    'echo "  Cua so nay GIU LAI de doc log. Tien trinh KHONG con chay."\n'
    'echo "  Log loi thuong nam ngay tren dong nay."\n'
    'echo "============================================================"\n'
)


def gnome_term(title, body, condition=None):
    """Run `body` in its own gnome-terminal window, kept open afterwards
    (exec bash) so logs stay readable on a crash.

    `body` is a shell script: either a plain string, or a list of pieces when it
    has to splice in a launch argument (ExecuteProcess concatenates the pieces of
    a nested list, which is the only way a LaunchConfiguration reaches bash -c).
    """
    tail = DIED_BANNER.format(title=title) + '\nexec bash'
    script = (body + [tail]) if isinstance(body, list) else [body + tail]
    return ExecuteProcess(
        prefix='gnome-terminal --title={} --'.format(title),
        cmd=['bash', '-c', script],
        output='screen',
        condition=condition,
    )


def generate_launch_description():
    # All windows open at once; ordering comes from the wait-loops, not timers.
    # The TF gets no window on purpose: it logs nothing and never fails in a way a
    # window would show -- you check it with `ros2 run tf2_ros tf2_echo map
    # d435i_depth_optical_frame`, not by reading a terminal.
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='hold_and_watch',
            description='hold_and_watch (default): take off and hold still facing '
                        'gate_1 -- use this to verify N1..N4. mission_gates: the '
                        'legacy FSM, currently broken (listens to the dead '
                        '/gate/obs) and only yaw-sweeps.',
        ),
        DeclareLaunchArgument(
            'controller_args', default_value='',
            description='extra flags for the controller. '
                        'controller_args:=--probe-accel runs hold_and_watch as the '
                        'accel probe (plan v2 R11 / stage 2): does this ArduCopter '
                        'honour the accel field of setpoint_raw? N5 is blocked on '
                        'the answer.',
        ),
        gnome_term('Gazebo', GAZEBO_CMD),
        gnome_term('SITL', SITL_CMD),
        gnome_term('DepthBridge', BRIDGE_CMD),
        gnome_term('MAVROS', MAVROS_CMD),
        gnome_term('Mission', MISSION_CMD),
        gnome_term('GateVision', VISION_CMD),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='camera_tf', arguments=CAMERA_TF_ARGS, output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='color_tf', arguments=COLOR_TF_ARGS, output='screen',
             parameters=[{'use_sim_time': True}]),
        DeclareLaunchArgument(
            'fsm', default_value='false',
            description='open N6 mission_fsm (fsm:=true). It judges PASSED by '
                        'the plan par. 7 R3 test and LANDS when the course is '
                        'done -- so only with a controller that flies the '
                        'gates, e.g. controller:=trajectory_follower.',
        ),
        gnome_term('GateTracker', TRACKER_CMD),
        gnome_term('GatePlanner', PLANNER_CMD),
        gnome_term('MissionFSM', FSM_CMD,
                   condition=IfCondition(LaunchConfiguration('fsm'))),
        gnome_term('PlanOverlay', OVERLAY_CMD),
    ])

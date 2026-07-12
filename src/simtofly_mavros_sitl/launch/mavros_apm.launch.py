#!/usr/bin/env python3
"""MAVROS for ArduPilot, with the `param` plugin denied.

This is `ros2 launch mavros apm.launch` with exactly one thing changed: the plugin
list. apm.launch HARDCODES its pluginlists_yaml and exposes no argument to override
it, so it cannot be reused -- but all it wraps is mavros/launch/node.launch, which is
a single Node with two params files. That is what this reproduces.

WHY THE PARAM PLUGIN IS DENIED

MAVROS dies mid-run, at random, with:

    terminate called after throwing an instance of 'std::future_error'
      what():  std::future_error: Promise already satisfied

The param plugin downloads all ~400 FCU parameters on connect. When a reply is slow it
re-requests; ArduPilot then answers both the original and the retry; the plugin fulfils
the same std::promise twice. That is an abort(), not a catchable error. The log says so
just before it goes:

    PR: request param #393 timeout, retries left 2, and 375 params still missing
    PR: got an unsolicited param value idx=397, not resetting retries count 3   (x N)

Replies are slow because the sim runs at about half real time -- RTF ~0.53, set by the
1000 Hz physics step and NOT by the cameras (measured both ways) -- which stretches the
MAVLink timesync RTT past a second. Fixing that properly means changing ArduPilot's FDM
rate, so we remove the plugin that cannot survive it instead.

Nothing in this repo calls /mavros/param/*; the missions use arming, set_mode, takeoff,
setpoint_raw and local_position. See config/mavros_pluginlists.yaml.

Not called directly -- sitl_mission.launch.py and gates_mission.launch.py run it:

    ros2 launch simtofly_mavros_sitl mavros_apm.launch.py fcu_url:=udp://:14550@
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pluginlists = os.path.join(
        get_package_share_directory('simtofly_mavros_sitl'),
        'config', 'mavros_pluginlists.yaml')
    # The per-plugin config stays ArduPilot's own -- we only change WHICH plugins load.
    apm_config = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url', default_value='udp://:14550@',
            description='SITL: udp://:14550@ . Hardware: see simtofly_mavros_real.',
        ),
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            output='screen',
            parameters=[
                pluginlists,
                apm_config,
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    # Empty = no GCS bridge. This is why we do not just shell out to
                    # node.launch: `gcs_url:=""` on the command line reaches ros2 launch
                    # as `gcs_url:=` and is rejected as malformed.
                    'gcs_url': '',
                    'tgt_system': 1,
                    'tgt_component': 1,
                    'fcu_protocol': 'v2.0',
                },
            ],
        ),
    ])

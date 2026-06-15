#!/usr/bin/env python3
"""
MAVROS Launch File for Real Hardware (Pixhawk via TELEM3)
----------------------------------------------------------
Author: Sidharth Mohan Nair
GitHub: https://github.com/sidharthmohannair

Connection: TELEM3 (SERIAL4) -> TX/RX/GND GPIO pins on companion computer
Device: /dev/ttyAMA0
Baud: 921600
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch MAVROS node configured for Pixhawk via TELEM3 GPIO UART.

    Prerequisites:
    1. Pixhawk SERIAL4_PROTOCOL = 2 (MAVLink2)
    2. Pixhawk SERIAL4_BAUD = 921600
    3. Wiring: TELEM3 TX -> RPi RX (GPIO15), TELEM3 RX -> RPi TX (GPIO14), GND -> GND
    4. RPi UART enabled: enable_uart=1 and dtoverlay=disable-bt in /boot/config.txt
    5. User in dialout group: sudo usermod -a -G dialout $USER
    """

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyAMA0:921600',
            description='FCU serial device and baud rate'
        ),

        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='GCS connection URL (optional)'
        ),

        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID'
        ),

        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID'
        ),

        DeclareLaunchArgument(
            'log_output',
            default_value='screen',
            description='Output destination for logs'
        ),

        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='MAVLink protocol version'
        ),

        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output=LaunchConfiguration('log_output'),
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': LaunchConfiguration('tgt_system'),
                'target_component_id': LaunchConfiguration('tgt_component'),
                'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            }]
        ),
    ])

#!/usr/bin/env python3

# Copyright (c) 2020-2023, Michael Ferguson
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    livox_config = os.path.join(
        get_package_share_directory('robomagellan'),
        'config', 'mid360.json'
    )

    return LaunchDescription([
        DeclareLaunchArgument('gen', default_value='gen1'),

        # Load the actual drivers for the appropriate robot
        IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    get_package_share_directory('robomagellan'),
                    'launch',
                    LaunchConfiguration('gen'),
                    '_drivers.launch.py']
                )
             ),
         ),

        # GPS publisher
        Node(
            name='nmea_serial_driver',
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            parameters=[{'baud': 9600,
                         'frame_id': 'base_link',
                         'port': '/dev/ttyACM0'}],
            remappings=[('fix', 'gps/fix'),
                        ('heading', 'gps/heading'),
                        ('vel', 'gps/vel')],
        ),

        # UM7 IMU
        Node(
            name='um7_driver',
            package='um7',
            executable='um7_node',
            parameters=[{'mag_updates': False,
                         'tf_ned_to_enu': False,
                         'update_rate': 100}],
            # This could be cleaned up when wildcards are implemented
            #   see https://github.com/ros2/rcl/issues/232
            remappings=[('imu/data', 'imu_um7/data'),
                        ('imu/mag', 'imu_um7/mag'),
                        ('imu/rpy', 'imu_um7/rpy'),
                        ('imu/temperature', 'imu_um7/temperature')]
        ),

        # Livox MID-360 Laser
        Node(
            name='livox',
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            output='screen',
            parameters=[{'user_config_path': livox_config,
                         'frame_id': 'livox_frame'}]
        ),

        # TODO: add mux between nav and joystick
        Node(
            name='joy',
            package='joy',
            executable='joy_node',
            parameters=[{'autorepeat_rate': 5.0}, ],
        ),

        # Teleop
        Node(
            name='teleop',
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[{'enable_button': 4,
                         'axis_linear.x': 4,
                         'scale_linear.x': 1.0,
                         'axis_angular.yaw': 0,
                         'scale_angular.yaw': 3.0} ],
            remappings=[('cmd_vel', 'base_controller/command')],
            output='screen',
        ),
    ])

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
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Load the URDF into a parameter
    bringup_dir = get_package_share_directory('robomagellan')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'robot.urdf.xacro')
    urdf = xacro.process(xacro_path)

    # Load the driver config
    driver_config = os.path.join(
        get_package_share_directory('robomagellan'),
        'config', 'etherbotix.yaml'
    )

    livox_config = os.path.join(
        get_package_share_directory('robomagellan'),
        'config', 'mid360.json'
    )

    return LaunchDescription([
        # Robot state publisher
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
        ),

        # Etherbotix drivers
        Node(
            name='etherbotix',
            package='etherbotix',
            executable='etherbotix_driver',
            parameters=[{'robot_description': urdf},
                        driver_config],
            remappings=[('odom', 'base_controller/odom')],
            output='screen',
        ),

        # GPS publisher
        Node(
            name='gps_publisher',
            package='etherbotix',
            executable='gps_publisher_node',
            parameters=[{'frame_id': 'base_link'}],
            remappings=[('nmea_sentence', 'gps/nmea_sentence')],
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
            parameters=[{"user_config_path": livox_config}]
        )

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
                         'axis_linear': 4,
                         'scale_linear': 1.0,
                         'axis_angular': 0,
                         'scale_angular': 3.0} ],
            remappings=[('cmd_vel', 'base_controller/command')],
            output='screen',
        ),
    ])

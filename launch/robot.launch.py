#!/usr/bin/env python3

# Copyright (c) 2020, Michael Ferguson
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
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

def launch_file(folder, name):
    # Get the path to an included launch file
    return os.path.join(
        get_package_share_directory('robomagellan'),
        'launch',
        folder,
        name
    )

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


    return LaunchDescription([
        # Arguments first
        DeclareLaunchArgument('offline', default_value='false'),

        # Robot state publisher
        Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            parameters=[{'robot_description': urdf}],
        ),

        # Drivers
        Node(
            name='etherbotix',
            package='etherbotix',
            node_executable='etherbotix_driver',
            parameters=[{'robot_description': urdf},
                        driver_config],
            remappings=[('odom', 'base_controller/odom')],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('offline')),
        ),
        Node(
            name='gps_publisher',
            package='etherbotix',
            node_executable='gps_publisher_node',
            parameters=[{'frame_id': 'base_link'}],
            remappings=[('nmea_sentence', 'gps/nmea_sentence')],
            condition=UnlessCondition(LaunchConfiguration('offline')),
        ),

        # Joystick Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file('drivers', 'teleop.launch.py')]),
            condition=UnlessCondition(LaunchConfiguration('offline')),
        ),
    ])


def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = generate_launch_description()
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()
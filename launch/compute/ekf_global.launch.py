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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('robomagellan')
    config_file = os.path.join(package_dir, 'config', 'ekf_global.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use clock topic (from bagfile) if true'),

        # EKF node for global odometry
        Node(
            name='ekf_global_odom',
            package='robot_localization',
            executable='ekf_node',
            parameters=[config_file,
                        {'use_sim_time': True}],
            # Subscriptions (in yaml):
            #   odom0: odometry/gps (from navsat node)
            #   odom1: base_controller/odom
            #   imu0:  imu/data
            # Publishes odometry/global
            remappings=[('odometry/filtered', 'odometry/global')]
        ),

        # Turn GPS fix into odometry message for EKF
        Node(
            name='navsat_transform_node',
            package='robot_localization',
            executable='navsat_transform_node',
            # yaw_offset - corrects to make IMU read 0 when facing East
            # magnetic_declination - see http://www.ngdc.noaa.gov/geomag-web
            # zero_altitude - pretends the world is flat
            # use_odometry_yaw - false, use IMU yaw
            parameters=[{'yaw_offset': 0.0,
                         'magnetic_declination_radians': 0.2516183,
                         'zero_altitude': True,
                         'publish_filtered_gps': False,
                         'frequency': 10.0,
                         'use_odometry_yaw': False}],
            # Subscribes to imu/data, gps/fix, odometry/global
            # Publishes gps/filtered, odometry/gps
            remappings=[('odometry/filtered', 'odometry/global')]
        )
    ])

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
    calibration = os.path.join(package_dir, 'config', 'imu_calibration.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use clock topic (from bagfile) if true'),

        Node(
            # Transforms imu data in imu_link frame to ENU data in base_link frame
            name='imu_data_transformer',
            package='imu_transformer',
            executable='imu_transformer_node',
            parameters=[{'target_frame': 'base_link',
                         'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('imu_in', 'imu_um7/data'),
                        ('mag_in', 'imu_um7/mag'),
                        ('imu_out', 'imu/data_raw'),
                        ('mag_out', 'imu/mag')]
        ),
        Node(
            # Filter raw gyro, accel and mag data into a usable orientation
            name='imu_filter',
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            parameters=[calibration,
                        {'orientation_stddev': 0.001,
                         'gain': 0.01,
                         'zeta': 0.001,
                         'publish_tf': False,
                         'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])

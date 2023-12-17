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

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri='magellan_f9p_2023_12_16', storage_id='mcap'),
    rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
)

writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri='magellan_f9p_2023_12_16_odom_fixed', storage_id='mcap'),
    rosbag2_py.ConverterOptions('', '')
)

topic_types = reader.get_all_topics_and_types()
for topic_type in topic_types:
    # Setup output bagfile
    writer.create_topic(topic_type)
    # Note the type if this is our TF data
    if topic_type.name == '/tf':
        tftypename = topic_type.type
    # Note the type if this is our odom data
    elif topic_type.name == "/base_controller/odom":
        odomtypename = topic_type.type

while reader.has_next():
    topic, data, timestamp = reader.read_next()
    filter_out = False
    # Filter out odom tf messages
    if topic == '/tf':
        msg_type = get_message(tftypename)
        msg = deserialize_message(data, msg_type)
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom':
                filter_out = True
                break
    elif topic == "/base_controller/odom":
        msg_type = get_message(odomtypename)
        msg = deserialize_message(data, msg_type)
        msg.pose.covariance[0] = -1
        msg.twist.covariance[0] = 0.1
        msg.twist.covariance[7] = 0.1
        msg.twist.covariance[35] = 10.0
        data = serialize_message(msg)

    # Copy over message if it isn't odom
    if not filter_out:
        writer.write(topic, data, timestamp)

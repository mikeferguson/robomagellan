#!/usr/bin/env python

"""
Copyright (c) 2014 Michael E. Ferguson. All right reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

_description = """
Convert a bagfile containing nmea_msgs/Fix into a geojson file.
"""

import argparse
import rospy
import rosbag
from robomagellan.geojson import Path

if __name__=="__main__":
    parser = argparse.ArgumentParser(description = _description)
    parser.add_argument('input', help='The bagfile to read nmea_msgs/Fix messages from.')
    parser.add_argument('output', help='The file to output.')
    parser.add_argument('--topic', help='Name of topic carrying nmea_msgs/Fix.', default="fix")
    parser.add_argument('--dist', help='Minimum distance between points (in meters).', default=2.0, type=float)
    args = parser.parse_args()

    path = Path(args.dist)

    bag = rosbag.Bag(args.input)
    print("Opened " + args.input)

    for topic, msg, t in bag.read_messages():
        if topic == args.topic:
            path.addPoint(msg.latitude, msg.longitude, msg.altitude)

    path.save(args.output)

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

import threading

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from robot_localization.srv import FromLL, ToLL

from robomagellan.utm import metric_dist

class gpsTracks:

    def __init__(self):
        # This is the raw track of pure gps data
        self.raw_track = []
        # This is our odometry tranformed into gps coordinates
        self.odom_track = []

        # Need TF2 to convert base_link to map
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        # Subscribe to GPS data
        self.gps_sub = rospy.Subscriber("gps/fix", NavSatFix, self.callback)

        # Service to convert to/from map frame
        self.from_ll = rospy.ServiceProxy('fromLL', FromLL)
        self.to_ll = rospy.ServiceProxy('toLL', ToLL)

        # Publisher
        self.pub = rospy.Publisher("/gps/tracks", MarkerArray, queue_size=1)

        # Publish in a thread
        self.thread = threading.Thread(target = self.run, args=())
        self.thread.daemon = True
        self.thread.start()

    # Proess incoming gps data
    def callback(self, msg):
        # Store the raw GPS track
        lla = [msg.latitude, msg.longitude, msg.altitude]
        try:
            if metric_dist(lla, self.raw_track[-1]) > 1.0:
                self.raw_track.append(lla)
                print(lla)
                self.odom_track.append(self.get_odom_point())
        except IndexError:
            # First iteration
            self.raw_track.append(lla)
            self.odom_track.append(self.get_odom_point())

    # Get the current pose of the robot after ekf correction
    def get_odom_point(self):
        # Convert base_link to map frame
        pt = PointStamped()
        pt.header.stamp = rospy.Time.now()
        pt.header.frame_id = "base_link"

        transformed = self.buffer.transform(pt, "map", timeout=rospy.Duration(0.5))

        resp = self.to_ll(transformed.point)
        return [resp.ll_point.latitude, resp.ll_point.longitude, resp.ll_point.altitude]

    # Get a marker of a GPS point, in map frame
    def get_marker(self, ns, id, fix, r, g, b):
        pt = GeoPoint()
        pt.latitude = fix[0]
        pt.longitude = fix[1]
        pt.altitude = fix[2]
        resp = self.from_ll(pt)

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.id = id
        marker.type = marker.SPHERE
        marker.pose.position.x = resp.map_point.x
        marker.pose.position.y = resp.map_point.y
        marker.pose.position.z = resp.map_point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.frame_locked = True

        return marker

    # Publish the track
    def publish(self):
        # Message to publish
        msg = MarkerArray()

        i = 0
        for fix in self.raw_track:
            marker = self.get_marker("track", i, fix, 1.0, 0.0, 0.0)
            msg.markers.append(marker)
            i += 1

        i = 0
        for fix in self.odom_track:
            marker = self.get_marker("odom", i, fix, 0.0, 0.0, 1.0)
            msg.markers.append(marker)
            i += 1

        self.pub.publish(msg)

    # Thread to publish
    def run(self):
        while not rospy.is_shutdown():
            self.publish()
            rospy.sleep(1.0)

if __name__ == "__main__":
    rospy.init_node("gps_tracks")
    tracks = gpsTracks()
    rospy.spin()

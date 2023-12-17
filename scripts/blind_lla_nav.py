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

from math import cos, sin, atan2
import time

# ROS2
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robomagellan.srv import FromLLA
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def get_relative_yaw(start, finish):
    dx = finish.position.x - start.position.x
    dy = finish.position.y - start.position.y
    return atan2(dy, dx)


class BlindNav(Node):

    # Cartesian coordinate goals
    goals = None

    def __init__(self):
        super().__init__('blind_nav')

        self.declare_parameter('lla_goals_file', rclpy.Parameter.Type.STRING)

        self.nav2_goal = None
        self.nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.from_lla_future = None
        self.from_lla_srv = self.create_client(FromLLA, 'from_lla')

        self.goals_msg = None
        self.goals_pub = self.create_publisher(MarkerArray, 'goals', 1)

        self.gps_pose = None
        self.gps_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'gps/pose',
                                                     self.gps_pose_cb, 1)

        self.timer = self.create_timer(1.0, self.control_loop)

    def control_loop(self):
        if not self.nav2_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Cannot navigate - nav2 action server not up')
            return

        self.get_logger().info('Control Loop')
        if self.gps_pose is None:
            self.get_logger().warn('Cannot navigate - waiting for GPS fix')
            return

        if self.nav2_goal is None:
            if self.update_goals():
                # No current goal, send next goal
                self.nav2_goal = NavigateToPose.Goal()
                self.nav2_goal.pose.header.frame_id = 'map'
                self.nav2_goal.pose.header.stamp = self.get_clock().now().to_msg()
                self.nav2_goal.pose.pose = self.goals[0]
                self.nav2_goal_future = self.nav2_action_client.send_goal_async(self.nav2_goal)
                self.nav2_goal_future.add_done_callback(self.nav2_goal_cb)
                self.get_logger().info('Sent nav2 goal')

        self.get_logger().info('Nav2 in process')
        time.sleep(1)

    # Convert goals to local cartesian frame
    # Returns true if there are goals to process
    def update_goals(self):
        if self.goals is None:
            if self.from_lla_future is None:
                # First time this is called
                # Load LLA goals from file
                filename = self.get_parameter('lla_goals_file').value
                with open(filename) as file:
                    self.lla_goals = yaml.safe_load(file)
                # Convert LLA goals into cartesian coordinates by calling service
                while not self.from_lla_srv.wait_for_service(timeout_sec=1.0):
                    self.get_logger().warn('Waiting for from_lla service')
                # Create request
                req = FromLLA.Request()
                for goal in self.lla_goals:
                    point = GeoPoint()
                    point.latitude = goal[0]
                    point.longitude = goal[1]
                    point.altitude = goal[2]
                    req.lla_points.append(point)
                print(req)
                # Call service
                self.from_lla_future = self.from_lla_srv.call_async(req)
                self.from_lla_future.add_done_callback(self.from_lla_cb)
            # Not ready to process yet
            return False

        # Goal message is valid, publish it
        self.goals_pub.publish(self.goals_msg)

        if len(self.goals) == 0:
            # We have completed all goals
            return False

        # Still have goals to pursue
        return True

    # Process the async return from service call
    def from_lla_cb(self, future):
        result = future.result()
        print(result)
        self.goals = []
        for point in result.map_points:
            msg = Pose()
            msg.position.x = point.x
            msg.position.y = point.y
            # Orientation is set from yaml file
            msg.orientation.z = sin(self.lla_goals[0][3] / 2.0)
            msg.orientation.w = cos(self.lla_goals[0][3] / 2.0)
            self.lla_goals = self.lla_goals[1:]
            self.goals.append(msg)
        print(self.goals)
        # Point intermediate goals at each other
        for i in range(len(self.goals) - 1):
            angle = get_relative_yaw(self.goals[i], self.goals[i + 1])
            self.goals[i].orientation.z = sin(angle / 2.0)
            self.goals[i].orientation.w = cos(angle / 2.0)
        print(self.goals)
        # Create visualization msg
        self.goals_msg = MarkerArray()
        goal_id = 0
        for goal in self.goals:
            msg = Marker()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.ns = 'goals'
            msg.id = goal_id
            goal_id += 1
            msg.type = msg.ARROW
            msg.action = msg.ADD
            msg.pose = goal
            msg.scale.x = 1.0
            msg.scale.y = 1.0
            msg.scale.z = 1.0
            msg.color.r = 1.0
            msg.color.a = 1.0
            self.goals_msg.markers.append(msg)
        # Publish new message
        self.goals_pub.publish(self.goals_msg)
        # Cleanup
        self.from_lla_future = None

    # Called when goal is accepted or rejected
    def nav2_goal_cb(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Nav2 goal was accepted')
            self.nav2_result_future = goal_handle.get_result_async()
            self.nav2_result_future.add_done_callback(self.nav2_result_cb)
        else:
            self.get_logger().error('Nav2 goal was rejected')
            # Clear the goal so we retry
            self.nav2_goal = None

    # Called when the goal is reached or failed
    def nav2_result_cb(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            # Succeeded in reaching goal, pop goal
            self.goals = self.goals[1:]
            self.get_logger().info('Nav2 succeeded')
        else:
            self.get_logger().error('Nav2 failed')
        # Regardless of status - clear the current goal
        self.nav2_goal = None

    # When a GPS fix occurs, the gps_to_cart node will start publishing this
    def gps_pose_cb(self, msg):
        self.gps_pose = msg


if __name__ == '__main__':
    rclpy.init()
    try:
        node = BlindNav()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

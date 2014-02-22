#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from topic_tools.srv import MuxSelect

class SimpleTeleop:

    def __init__(self):
        self.axis_linear = rospy.get_param("axis_linear", 3)
        self.axis_angular = rospy.get_param("axis_angular", 0)
        self.axis_deadman = rospy.get_param("axis_deadman", 10)
        self.scale_linear = rospy.get_param("scale_linear", 0.5)
        self.scale_angular = rospy.get_param("scale_angular", 0.5)
        self.last_message = rospy.Time(0)
        self.twist = Twist()

        self._service = None
        if rospy.get_param("use_mux", True):
            rospy.loginfo('Waiting for cmd_vel_mux/select service to be available...')
            rospy.wait_for_service('cmd_vel_mux/select')
            self._service = rospy.ServiceProxy('cmd_vel_mux/select', MuxSelect)
            rospy.loginfo('...connected')

        self._vel_pub = rospy.Publisher("teleop/cmd_vel", Twist)
        self._joy_sub = rospy.Subscriber("joy", Joy, self.joyCb)

    def joyCb(self, msg):
        self.deadman_pressed = msg.buttons[self.axis_deadman] > 0
        self.twist.linear.x = self.scale_linear * msg.axes[self.axis_linear]
        self.twist.angular.z = self.scale_angular * msg.axes[self.axis_angular]
        self.last_message = rospy.Time.now()

    def run(self):
        active = False
        prev_topic = ''

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # are we ok?
            if rospy.Time.now() - self.last_message > rospy.Duration(0.5):
                self.deadman_pressed = False

            if self.deadman_pressed:
                # are we active already or just starting:
                if not active and self._service:
                    try:
                        resp = self._service('teleop/cmd_vel')
                        prev_topic = resp.prev_topic
                    except rospy.ServiceException as exc:
                        rospy.logerr("Service did not process request: " + str(exc))
                # publish regular command
                self._vel_pub.publish(self.twist)
                active = True

            elif active:
                # deadman let up, stop
                self._vel_pub.publish(Twist())
                # disconnect mux
                if self._service:
                    try:
                        resp = self._service(prev_topic)
                    except rospy.ServiceException as exc:
                        rospy.logerr("Service did not process request: " + str(exc))
                active = False

            r.sleep()

if __name__=="__main__":
    rospy.init_node("teleop")
    teleop = SimpleTeleop()
    teleop.run()

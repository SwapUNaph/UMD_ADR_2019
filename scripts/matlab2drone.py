#!/usr/bin/env python

# Script developed by Sharon Shallom

import rospy
import signal
import sys
from std_msgs.msg import Bool, Int32, Empty
from geometry_msgs.msg import Twist
import numpy as np


def signal_handler(_, __):
    sys.exit(0)


def callback_button_pressed(button):
    # Takeoff
    if button.data == 7:
        if not cmd[2] == 0:
            rospy.loginfo('Warning Throttle Not Centered')
            rospy.loginfo(cmd[2])
        elif not cmd[0] == 0:
            rospy.loginfo('Warning Pitch Not Centered')
            rospy.loginfo(cmd[0])
        elif not cmd[1] == 0:
            rospy.loginfo('Warning Roll Not Centered')
            rospy.loginfo(cmd[1])
        elif not cmd[3] == 0:
            rospy.loginfo('Warning Yaw Not Centered')
            rospy.loginfo(cmd[3])
        else:
            rospy.loginfo('Takeoff Initiated')
            publisher_takeoff.publish(Empty())
    # Land
    if button.data == 8:
        publisher_land.publish(Empty())
        rospy.loginfo('Landing Initiated')
    # reset
    if button.data == 9:
        publisher_reset.publish(Empty())
        rospy.loginfo('Reset Initiated')


def callback_update_cmd(data):
    global cmd
    cmd = np.array([data.linear.x, data.linear.y, data.linear.z, data.angular.z])


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('control_manual')

    # Global variables
    cmd = np.array([0, 0, 0, 0])

    # Publishers
    publisher_takeoff = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1, latch=True)
    publisher_land = rospy.Publisher("/bebop/land", Empty, queue_size=1, latch=True)
    publisher_reset = rospy.Publisher("/bebop/reset", Empty, queue_size=1, latch=True)
    publisher_cmd = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/auto/flt_st_cmd", Int32, callback_button_pressed)
    rospy.Subscriber("/auto/cmd", Twist, callback_update_cmd)

    rospy.loginfo("ready")
    rospy.loginfo("Btn Code:")
    rospy.loginfo("7 - Takeoff")
    rospy.loginfo("8 - Land")
    rospy.loginfo("9 - Reset")
    rospy.loginfo("10 - N/A")
    rospy.loginfo("11 - Manual Mode")
    rospy.loginfo("12 - Autonomous Mode")

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        msg = Twist()
        msg.linear.x = cmd[0]
        msg.linear.y = cmd[1]
        msg.linear.z = cmd[2]
        msg.angular.z = cmd[3]
        publisher_cmd.publish(msg)
    rate.sleep()
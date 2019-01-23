#!/usr/bin/env python

# Script developed by Sharon Shallom

import rospy
import signal
import sys
from std_msgs.msg import Bool, Int32, Empty
# import numpy as np
# from bebop_tools/config import bebop_tools
# roslaunch bebop_tools joy_teleop.launch


def signal_handler(_, __):
    sys.exit(0)


def callback_button_pressed(button):
    # rospy.loginfo(button)
    # Takeoff
    if button.data == 7:
        rospy.loginfo('Takeoff Initiated')
        publisher_takeoff.publish(msg)

    # Land
    if button.data == 8:
        publisher_land.publish(msg)
        rospy.loginfo('Landing Initiated')
    # reset
    if button.data == 9:
        publisher_reset.publish(msg)
        rospy.loginfo('Reset Initiated')


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('control_manual')

    # Global variables
    msg = Empty()

    # Publishers
    # publisher_flt_st_cmd = rospy.Publisher("/auto/flt_st_cmd", Auto_Driving_Msg, queue_size=1, latch=True)j
    publisher_takeoff = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1, latch=True)
    publisher_land = rospy.Publisher("/bebop/land", Empty, queue_size=1, latch=True)
    publisher_reset = rospy.Publisher("/bebop/reset", Empty, queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/auto/flt_st_cmd", Int32, callback_button_pressed)

    rospy.loginfo("ready")

    rospy.spin()

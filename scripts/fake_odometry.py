#!/usr/bin/env python

# Script developed by Sharon Shallom

import rospy
# from geometry_msgs.msg import Twist
import signal
import sys
# from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
# from std_msgs.msg import Bool, Int32
# from bebop_auto.msg import Auto_Driving_Msg
from nav_msgs.msg import Odometry
import time


def signal_handler(_, __):
    sys.exit(0)


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('fake_odometry')

    # Global variables

    # Publishers
    fake_odometry_pub = rospy.Publisher('/auto/fake_odom', Odometry, queue_size=1, latch=True)

    # run with 20Hz
    rate = rospy.Rate(20)

    rospy.loginfo("ready")
    rospy.loginfo("publishing fake odometry")

    while True:
        rate.sleep()
        msg = Odometry()
        msg.twist.twist.linear.x = time.time() - 1547658499
        fake_odometry_pub.publish(msg)
        rospy.loginfo(msg.twist.twist.linear)



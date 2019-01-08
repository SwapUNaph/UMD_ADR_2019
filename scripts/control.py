#!/usr/bin/env python

# Script developed by Sharon Shallom

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import signal
import sys
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from std_msgs.msg import Bool, Int32
from bebop_auto.msg import Auto_Driving_Msg
# import common_resources as cr
from nav_msgs.msg import Odometry


def signal_handler(_, __):
    sys.exit(0)

def callback_bebop_odometry_changed(data):
    odometry = data
    rospy.loginfo('update')
    rospy.loginfo('Twist x = ' + str(data.twist.twist.linear.x))

if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('control')

    # Global variables

    # Publishers
    # cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/auto/fake_odom", Odometry, callback_bebop_odometry_changed)


    rospy.loginfo("ready")

    rospy.spin()
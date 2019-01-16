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
import numpy as np


def signal_handler(_, __):
    sys.exit(0)


def callback_bebop_odometry_changed(data):
    odometry = data
    rospy.loginfo('update')
    drone_pos = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
    drone_vel = np.array([1, 0, 0])
    SP_pos = np.array([1, 1, 0])
    SP_vel = np.array([2, 2, 0])

    # create path frame
    path_frame_x = make_unit(SP_vel)
    path_frame_y = make_unit(np.array([-SP_vel[1], SP_vel[0], 0]))
    path_frame_z = make_unit( np.array([-SP_vel[0]*SP_vel[2], -SP_vel[1]*SP_vel[2], (SP_vel[0]*SP_vel[0])+(SP_vel[1]*SP_vel[1])]))
    path_frame_matrix = np.matrix((path_frame_x, path_frame_y, path_frame_z))

    # Velocity controller
    delta_pos_global = SP_pos - drone_pos
    rotation_matrix = np.linalg.inv(path_frame_matrix)
    # multiply rotation matrix with delta position global
    delta_pos_trajectory = delta_pos_global*rotation_matrix
    delta_vel_trajectory = 1*delta_pos_trajectory
    # multiply SP vel by path frame: WP velocity trajectory
    WP_vel_traj = SP_vel*rotation_matrix
    # find desired velocity trajectory
    des_trajectory = delta_vel_trajectory + WP_vel_traj
    # Drone velocity trajectory
    drone_vel_traj = drone_vel*rotation_matrix
    # Delta velocity desired trajectory
    delta_vel_des_trajectory = des_trajectory - drone_vel_traj
    # calculate acceleration, (Vel Controller X,Y1)
    accel_traj = delta_vel_des_trajectory # PID(s)







def make_unit(vector):
    unit_vector = vector/np.linalg.norm(vector)
    return unit_vector



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
#!/usr/bin/env python

# Script developed by Sharon Shallom

import rospy
# from geometry_msgs.msg import Twist
import signal
import sys
# from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
# from std_msgs.msg import Bool, Int32
from bebop_auto_2019.msg import Auto_Driving_Msg
from nav_msgs.msg import Odometry
import numpy as np
import math


def signal_handler(_, __):
    sys.exit(0)


def callback_bebop_odometry_changed(data):
    odometry = data
    rospy.loginfo('update')
    drone_pos = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z])
    drone_vel = np.array([1, 0, 0])
    drone_ori = np.array([0, 0, np.pi])
    SP_pos = np.array([1, 1, 0])
    SP_vel = np.array([2, 2, 0])
    g = 9.81
    SP_yaw = np.pi + 0.1
    max_theta_X = np.pi/2
    max_theta_Y = np.pi/2
    max_theta_Z = 1
    max_rot_Z = np.pi/3

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
    # Calculating global acceleration through matrix multiplication
    accel_global = np.matmul(accel_traj, np.linalg.inv(path_frame_matrix))

    # Using Drone orientation to calculate drone yaw
    drone_yaw = drone_ori[2]
    # Orientation to 3D yaw rotation
    yaw = np.array([drone_yaw, 0, 0])
    # Rotation angles to direction cosine matrix
    rot_matrix = euler_matrix(yaw[2], yaw[1], yaw[0])
    # Calculating acceleration of drone through matrix multiplication
    accel_drone = np.array(accel_global*rot_matrix)
    # Factor in gravity
    pitch_des = accel_drone[0][0]/g
    roll_des = accel_drone[0][1]/g
    yaw_des = accel_drone[0][2]

    # Find difference between desired yaw and drone yaw
    delta_yaw = SP_yaw - drone_yaw
    # Hdg controller R
    d_yaw = np.arctan2(np.sin(delta_yaw), np.cos(delta_yaw))
    # Put in PID(s) when get there
    yaw_rate = d_yaw # PID(s)
    # Block Parameters
    ctrl_x = pitch_des*(1/max_theta_X)
    ctrl_y = roll_des*(1/max_theta_Y)
    ctrl_z = yaw_des*(1/max_theta_Z)
    ctrl_r = yaw_rate*(1/max_rot_Z)

    msg = Auto_Driving_Msg()
    msg.x = ctrl_x
    msg.y = ctrl_y
    msg.z = ctrl_z
    msg.r = ctrl_r

    publisher_auto_drive.publish(msg)


# Transform euler angle into rotation matrix (ZYX)
def euler_matrix(attitude, bank, heading):
    sa = np.sin(attitude)
    ca = np.cos(attitude)
    sb = np.sin(bank)
    cb = np.cos(bank)
    sh = np.sin(heading)
    ch = np.cos(heading)
    rotation = np.matrix([[ch*ca, -ch*sa*cb+sh*sb, ch*sa*sb+sh*cb], [sa, ca*cb, -ca*sb], [-sh*ca, sh*sa*cb+ch*sb, -sh*sa*sb+ch*cb]])
    return rotation


def make_unit(vector):
    unit_vector = vector/np.linalg.norm(vector)
    return unit_vector


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('control')

    # Global variables

    # Publishers
    publisher_auto_drive = rospy.Publisher("/auto/auto_drive", Auto_Driving_Msg, queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/auto/fake_odom", Odometry, callback_bebop_odometry_changed)

    rospy.loginfo("ready")

    rospy.spin()
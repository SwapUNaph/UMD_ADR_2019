#!/usr/bin/env python

# Script developed by Vincenz Frenzel
#  --- Changelog ---
# Goal:     Input by bebop camera and stereo camera and state machine and merged_odometry. Depending on state machine, use algorithms to detect gates on the image. Afterwards, position gates on a global map based on the odometry as gate orientation matrices are relative to camera orientation. Output position and orientation of next gate
# Status:   06/19:  Uses cheap stereo camera left video stream to identify gate position and orientation. Does not incorporate any odometry and publishes gate position at (1,0,0) with 3 Hz. Coordainte transformation missing. CV algorithm might need to be improved if there is a performance issue
#           07/06: Publishes gate position as seen from the camera (tvec: translation from camera, rvec: rotation from camera, and applicable bebop odometry
#           11/03: Commented code
#           02/15: edited for 2019

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import math
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Bool, Float32

import signal
import sys


def signal_handler(_, __):
    sys.exit(0)


# intersect two line bundles to one single intersection point
def isect_lines_bundle(lines1, lines2, start, end):
    # start and end points of all lines
    x1 = start[0, lines1]
    y1 = start[1, lines1]
    x2 = end[0, lines1]
    y2 = end[1, lines1]
    x3 = start[0, lines2]
    y3 = start[1, lines2]
    x4 = end[0, lines2]
    y4 = end[1, lines2]

    # create matrices from lists to enable fast numpy operations
    x1 = np.repeat(np.matrix(x1, dtype='float'), len(lines2), axis=0)
    y1 = np.repeat(np.matrix(y1, dtype='float'), len(lines2), axis=0)
    x2 = np.repeat(np.matrix(x2, dtype='float'), len(lines2), axis=0)
    y2 = np.repeat(np.matrix(y2, dtype='float'), len(lines2), axis=0)
    x3 = np.transpose(np.repeat(np.matrix(x3, dtype='float'), len(lines1), axis=0))
    y3 = np.transpose(np.repeat(np.matrix(y3, dtype='float'), len(lines1), axis=0))
    x4 = np.transpose(np.repeat(np.matrix(x4, dtype='float'), len(lines1), axis=0))
    y4 = np.transpose(np.repeat(np.matrix(y4, dtype='float'), len(lines1), axis=0))

    # basic idea: g1=p1+t*(p2-p1)
    #             g2=p3+s*(p4-p3) and solve for s
    s = (np.multiply(x2 - x1, y3 - y1) - np.multiply(x3 - x1, y2 - y1)) / (np.multiply(x4 - x3, y2 - y1) - np.multiply(
        x2 - x1, y4 - y3))

    x = x3 + np.multiply(s, x4 - x3)
    y = y3 + np.multiply(s, y4 - y3)

    # calculate mean of all intersection not taking into account invalid entries
    return np.ma.masked_invalid(x).mean(), np.ma.masked_invalid(y).mean()


# mask an hsv image with a specific color
def mask_image(hsv):
    # make this function compatible also for pointer detection -> color information required

    lower_color = orange_low
    upper_color = orange_high

    publisher = publisher_image_threshold_orange

    # mask image
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Bitwise-AND mask and original image only for fun
    # global image_pub_dev1

    # output_im = bridge.cv2_to_imgmsg(mask, encoding="8UC1")
    # image_pub_dev1.publish(output_im)

    # show = cv2.resize(res,(1280,720))
    # cv2.imshow("show",show)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    exit()

    # output masked image. fast if only mask is returned, more informative but much slower if mask is applied onto rgb
    # and then returned. therefore uncomment the first three lines, comment our the following two
    global bridge
    # rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    # res = cv2.bitwise_and(rgb, rgb, mask=mask)
    # output_im = bridge.cv2_to_imgmsg(res, encoding="rgb8")
    output_im = cv2.resize(mask, (0, 0), fx=output_scale, fy=output_scale)
    output_im = bridge.cv2_to_imgmsg(output_im, encoding="8UC1")
    publisher.publish(output_im)

    # display image
    # cv2.imshow("test",res)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    exit()

    return mask


# callback of a new image
def stereo_callback(data):
    global bridge
    global latest_pose
    global rvec
    global tvec

    # set a debug mode where this runs without pose and can also run special detection if required
    debug_on = False
    if debug_on:
        this_pose = Pose()
    else:
        if latest_zed_pose is None or latest_bebop_pose is None:
            print("No position")
            return
        this_pose = latest_zed_pose
        this_pose.position.z = latest_bebop_pose.position.z

    # convert image msg to matrix
    rgb = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)

    # convert image to HSV
    # original image is BGR but conversion here is RGB so red color does not wrap around, saves runtime
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

    # masking image
    mask = mask_image(hsv)

    # probabilistic hough transform
    min_line_length = 720/4
    max_line_gap = 30

    # hough lines, returns start and end points of lines. lines are sorted by num of votes, but that number is unknown
    # image, precision r, precision theta, number of votes, minLineLength to consider, maxLineGap to overjump
    lines = cv2.HoughLinesP(mask, 5, np.pi / 90, 300, minLineLength=min_line_length, maxLineGap=max_line_gap)

    # print DEBUG onto image
    if debug_on:
        cv2.putText(rgb, "DEBUG", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 3.5, (0, 0, 255), 10, cv2.LINE_AA)

    # if there are less than 2 lines, exit
    # before exit: publish empty result and publish current rgb image
    if lines is None or len(lines) < 2:
        rospy.loginfo("no lines")
        publisher_result.publish(Odometry())
        rgb = cv2.resize(rgb, (0, 0), fx=output_scale, fy=output_scale)
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        publisher_image_gate.publish(output_im)
        return

    # lines have been found
    # angles = []

    # shorten list of lines to only use good matches
    if len(lines) > 40:
        # if more than 40: use best 40 normally, use best 80 for special jungle detection (two gates need to be found)
        lines = lines[:40]

    # display lines and end points in rgb image, color dots by number of votes
    # votes = np.array(list(reversed(range(len(lines))))) + 1
    # for counter, line in enumerate(lines):
    #     for x1, y1, x2, y2 in line:
    #         # angles.append(math.atan2(y2 - y1, x2 - x1) * 180 / np.pi)  # between -90 and 90
    #         cv2.circle(rgb, (x1, y1), 5, (votes[counter]*255.0/len(lines), votes[counter]*255.0/len(lines),
    #           votes[counter]*255.0/len(lines)), 2)
    #         cv2.circle(rgb, (x2, y2), 5, (votes[counter]*255.0/len(lines), votes[counter]*255.0/len(lines),
    #           votes[counter]*255.0/len(lines)), 2)
    #         # cv2.line(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # plot histogram of angles. usually angles are not calculated though
    # plt.clf()
    # hist = np.histogram(angles, 90, [-90.0, 90.0])
    #
    # ax = plt.axes()
    # ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
    # ax.yaxis.set_minor_locator(ticker.MultipleLocator(100))
    # plt.plot(hist[1][1:], hist[0], 'bo-')
    # plt.axis([-90, 90, 0, 10])
    # # x_ticks = np.arange(-90, 90, 5)
    # # y_ticks = np.arange(-1500, 1500, 100)
    # # ax.set_xticks(x_ticks)
    # # ax.set_yticks(y_ticks)
    # plt.grid(which='both')

    # cluster start and end points
    # how big are clusters (pixels)
    dist_thresh = 80

    # array of lines
    lines = np.array(lines)
    # squeeze empty dimensions
    lines = np.squeeze(lines)
    # get corners instead of lines
    corners = np.transpose(lines)
    # seperate start and end points
    start = corners[:2, :]
    end = corners[2:, :]
    # append start and end points to each other
    start_end = np.concatenate((start, end), axis=1)
    # create fake number for votes to weigh good lines better
    votes = np.array(list(reversed(range(len(lines)))))+1
    # concatenate votes so it's as long as start+end list
    votes = np.concatenate((votes, votes))
    # set up variables for first clustering
    x_ms_1 = np.array([])       # list of x coords of cluster centers
    y_ms_1 = np.array([])       # list of y
    vote_ms_1 = np.array([])    # list of number of votes

    # draw all start and end points
    # for i in range(np.shape(start_end)[1]):
    #     cv2.circle(rgb, (int(start_end[0][i]), int(start_end[1][i])), 5, (0, 0, 255), 2)

    # start clustering
    # nothing is clustered yet -> zeros
    clusters_1 = np.zeros(np.shape(start_end)[1])
    # number of current cluster
    cluster = 0
    # while not every point is in a cluster: keep clustering
    while not clusters_1.all():
        # start with cluster number 1
        cluster = cluster+1
        # get first unclustered index
        s = np.where(clusters_1 == 0)[0][0]
        # get position and votes and set as average for cluster
        x_m = start_end[0][s]
        y_m = start_end[1][s]
        vote_m = votes[s]
        # set this point as clustered
        clusters_1[s] = cluster
        # find points belonging to this cluster until no more were found (and go_around becomes false)
        go_around = True

        while go_around:
            go_around = False  # assume there will not be another point
            for idx in np.where(clusters_1 == 0)[0]:
                # go through all unclustered points and calculate distance to cluster center
                x = start_end[0][idx]
                y = start_end[1][idx]
                vote = votes[idx]
                distance = math.sqrt((x-x_m)**2+(y-y_m)**2)
                # print distance
                # check distance between point and cluster mean
                if distance < dist_thresh:
                    # enable go around
                    go_around = True
                    # recalculate average and votes
                    x_m = (x_m * vote_m + x * vote) / (vote_m + vote)
                    y_m = (y_m * vote_m + y * vote) / (vote_m + vote)
                    vote_m = vote_m + vote
                    # set point as clustered
                    clusters_1[idx] = cluster

        # no more points were found. add average to a list of clusters
        x_ms_1 = np.append(x_ms_1, x_m)
        y_ms_1 = np.append(y_ms_1, y_m)
        vote_ms_1 = np.append(vote_ms_1, vote_m)

    # weigh cluster votes, so that it is better the higher in the image the cluster is (build gate from top to bottom)
    # 1000 only used to make numbers smaller
    # commented out: also use horizontal position (weigh clusters in center higher)
    idx = vote_ms_1 * (800-y_ms_1) / 1000 * (400-abs(x_ms_1-1280/2))/100

    # no cluster was created -> exit
    if cluster == 0:
        rospy.loginfo("empty sequence 1")
        publisher_result.publish(Odometry())
        rgb = cv2.resize(rgb, (0, 0), fx=output_scale, fy=output_scale)
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        publisher_image_gate.publish(output_im)
        return

    # draw all clusters with their weighted votes
    # for index in range(len(vote_ms_1)):
    #     cv2.circle(rgb, (int(x_ms_1[index]), int(y_ms_1[index])), dist_thresh, (255, 255, 255), 2)
    #     cv2.putText(rgb, str(int(idx[index])), (int(x_ms_1[index]), int(y_ms_1[index])), cv2.FONT_HERSHEY_SIMPLEX, 1,
    #                   (255, 255, 255), 2, cv2.LINE_AA)

    # find strongest corner
    max_cluster = np.argmax(idx)+1

    # set up list of all corner points, these will actually not be used for solvePnP algorithm
    corner_points = np.zeros((5, 2))
    # add point to gate corner
    corner_points[0, :] = [x_ms_1[max_cluster - 1], y_ms_1[max_cluster - 1]]

    # draw corner
    # cv2.circle(rgb, (int(corner_points[0, 0]), int(corner_points[0, 1])), dist_thresh, (255, 0, 0), 2)
    # cv2.circle(rgb, (int(corner_points[0, 0]), int(corner_points[0, 1])), 2, (255, 0, 0), 2)

    # find all lines originating from this first cluster
    to_cluster_id = np.where(clusters_1 == max_cluster)[0]
    to_cluster_id1 = to_cluster_id[to_cluster_id < len(lines)]  # lines that started in cluster
    to_cluster_id2 = to_cluster_id[to_cluster_id >= len(lines)] - len(lines)  # lines that ended in cluster
    lines_cluster_1 = np.concatenate((to_cluster_id1, to_cluster_id2))  # append both

    # draw the lines that were found and will be reclustered
    # for line in to_cluster_id1:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (0, 0, 255), 2)
    # for line in to_cluster_id2:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (0, 0, 255), 2)

    # use endpoints of the lines that started in cluster
    to_cluster1 = end[:, to_cluster_id1]
    # and startpoints of lines that ended in cluster
    to_cluster2 = start[:, to_cluster_id2]
    # append the two
    to_cluster = np.concatenate((to_cluster1, to_cluster2), axis=1)
    # find their votes
    votes_2 = np.concatenate((votes[to_cluster_id1], votes[to_cluster_id2]))

    # see above
    clusters_2 = np.zeros(np.shape(to_cluster)[1])
    cluster = 0
    x_ms_2 = np.array([])
    y_ms_2 = np.array([])
    vote_ms_2 = np.array([])

    while not clusters_2.all():
        cluster = cluster+1
        s = np.where(clusters_2 == 0)[0][0]
        x_m = to_cluster[0][s]
        y_m = to_cluster[1][s]
        vote_m = votes_2[s]
        clusters_2[s] = cluster

        for idx in np.where(clusters_2 == 0)[0]:
            x = to_cluster[0][idx]
            y = to_cluster[1][idx]
            vote = votes_2[idx]
            distance = math.sqrt((x-x_m)**2+(y-y_m)**2)
            # print distance
            if distance < dist_thresh:
                x_m = (x_m * vote_m + x * vote) / (vote_m + vote)
                y_m = (y_m * vote_m + y * vote) / (vote_m + vote)
                vote_m = vote_m + vote
                clusters_2[idx] = cluster

        x_ms_2 = np.append(x_ms_2, x_m)
        y_ms_2 = np.append(y_ms_2, y_m)
        vote_ms_2 = np.append(vote_ms_2, vote_m)

    # not two clusters were created from the points -> exit or run jungle special detection if it was enabled
    if cluster < 2:
        rospy.loginfo("empty sequence 2")
        publisher_result.publish(Odometry())
        rgb = cv2.resize(rgb, (0, 0), fx=output_scale, fy=output_scale)
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        publisher_image_gate.publish(output_im)
        return

    # find two strongest clusters
    maximum_ids = vote_ms_2.argsort()[-2:][::-1]

    # these two clusters have to be matched to the original clusters of #1 as only there we know which lines contributes
    # therefore calculate distance of cluster 2.1 to all others and find minimum
    x_diff = x_ms_1 - x_ms_2[maximum_ids[0]]
    y_diff = y_ms_1 - y_ms_2[maximum_ids[0]]
    diff = x_diff * x_diff + y_diff * y_diff
    # a special algorithm can also take into account how strong the cluster is. In special cases, not the clostest
    # cluster is the one we are looking for but a further one that is much stronger (but disabled)
    # temp_vote = vote_ms_1.copy()
    # for index in range(len(temp_vote)):
    #     if diff[index] > relocate_factor * relocate_factor * dist_thresh * dist_thresh:
    #         temp_vote[index] = 0
    # close_id1 = np.argmax(temp_vote)
    # find minimum
    close_id1 = np.argmin(diff)

    # see above but for 2.2 to all others
    x_diff = x_ms_1 - x_ms_2[maximum_ids[1]]
    y_diff = y_ms_1 - y_ms_2[maximum_ids[1]]
    diff = x_diff * x_diff + y_diff * y_diff
    # temp_vote = vote_ms_1.copy()
    # for index in range(len(temp_vote)):
    #     if diff[index] > relocate_factor * relocate_factor * dist_thresh * dist_thresh:
    #         temp_vote[index] = 0
    # close_id2 = np.argmax(temp_vote)
    close_id2 = np.argmin(diff)

    # add to list although the corners will never be used
    corner_points[1, :] = [x_ms_1[close_id1], y_ms_1[close_id1]]
    corner_points[2, :] = [x_ms_1[close_id2], y_ms_1[close_id2]]

    # draw these two points
    # cv2.circle(rgb, (int(corner_points[1, 0]), int(corner_points[1, 1])), dist_thresh, (0, 255, 0), 2)
    # cv2.circle(rgb, (int(corner_points[2, 0]), int(corner_points[2, 1])), dist_thresh, (0, 255, 0), 2)
    # cv2.circle(rgb, (int(corner_points[1, 0]), int(corner_points[1, 1])), 2, (0, 255, 0), 2)
    # cv2.circle(rgb, (int(corner_points[2, 0]), int(corner_points[2, 1])), 2, (0, 255, 0), 2)

    # find lines originating from these two median points and recluster its endpoints into two separate clusters
    close_id1 = close_id1 + 1  # to equal cluster number
    close_id2 = close_id2 + 1  # to equal cluster number

    # find lines in closest clusters
    lines_cluster_2a_long = np.where(clusters_1 == close_id1)[0]
    lines_cluster_2b_long = np.where(clusters_1 == close_id2)[0]

    # do the start and end point thing again (subtract number of lines from all points in second part of start_end_list
    to_cluster_id1_a = lines_cluster_2a_long[lines_cluster_2a_long < len(lines)]
    to_cluster_id2_a = lines_cluster_2a_long[lines_cluster_2a_long >= len(lines)] - len(lines)
    to_cluster_id1_b = lines_cluster_2b_long[lines_cluster_2b_long < len(lines)]
    to_cluster_id2_b = lines_cluster_2b_long[lines_cluster_2b_long >= len(lines)] - len(lines)

    # concatenate
    lines_cluster_2a = np.concatenate((to_cluster_id1_a, to_cluster_id2_a))
    lines_cluster_2b = np.concatenate((to_cluster_id1_b, to_cluster_id2_b))

    # find which lines were used in last clustering
    rm1a = np.in1d(to_cluster_id1_a, to_cluster_id1) + np.in1d(to_cluster_id1_a, to_cluster_id2)
    rm2a = np.in1d(to_cluster_id2_a, to_cluster_id1) + np.in1d(to_cluster_id2_a, to_cluster_id2)
    rm1b = np.in1d(to_cluster_id1_b, to_cluster_id1) + np.in1d(to_cluster_id1_b, to_cluster_id2)
    rm2b = np.in1d(to_cluster_id2_b, to_cluster_id1) + np.in1d(to_cluster_id2_b, to_cluster_id2)

    # remove them from this clustering as they don't go to point 4 but back to point 1
    to_cluster_id1_a = np.delete(to_cluster_id1_a, np.where(rm1a))
    to_cluster_id2_a = np.delete(to_cluster_id2_a, np.where(rm2a))
    to_cluster_id1_b = np.delete(to_cluster_id1_b, np.where(rm1b))
    to_cluster_id2_b = np.delete(to_cluster_id2_b, np.where(rm2b))

    # draw all lines that will be considered now
    # for line in to_cluster_id1_a:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (0, 0, 0), 2)
    # for line in to_cluster_id2_a:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (0, 0, 0), 2)
    #
    # for line in to_cluster_id1_b:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 0, 0), 2)
    # for line in to_cluster_id2_b:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 0, 0), 2)

    # do two seperate clusterings now
    # cluster a
    to_cluster1 = end[:, to_cluster_id1_a]
    to_cluster2 = start[:, to_cluster_id2_a]
    to_cluster = np.concatenate((to_cluster1, to_cluster2), axis=1)

    votes_3a = np.concatenate((votes[to_cluster_id1_a], votes[to_cluster_id2_a]))
    #
    # for i in range(np.shape(to_cluster)[1]):
    #     cv2.circle(rgb, (int(to_cluster[0][i]), int(to_cluster[1][i])), 2, (255, 0, 0), 2)

    clusters_3a = np.zeros(np.shape(to_cluster)[1])
    cluster = 0
    x_ms_3a = np.array([])
    y_ms_3a = np.array([])
    vote_ms_3a = np.array([])

    while not clusters_3a.all():
        cluster = cluster + 1
        s = np.where(clusters_3a == 0)[0][0]
        x_m = to_cluster[0][s]
        y_m = to_cluster[1][s]
        vote_m = votes_3a[s]
        clusters_3a[s] = cluster

        for idx in np.where(clusters_3a == 0)[0]:
            x = to_cluster[0][idx]
            y = to_cluster[1][idx]
            vote = votes_3a[idx]
            distance = math.sqrt((x - x_m) ** 2 + (y - y_m) ** 2)
            # print distance
            if distance < dist_thresh:
                x_m = (x_m * vote_m + x * vote) / (vote_m + vote)
                y_m = (y_m * vote_m + y * vote) / (vote_m + vote)
                vote_m = vote_m + vote
                clusters_3a[idx] = cluster

        x_ms_3a = np.append(x_ms_3a, x_m)
        y_ms_3a = np.append(y_ms_3a, y_m)
        vote_ms_3a = np.append(vote_ms_3a, vote_m)

    if cluster == 0:
        rospy.loginfo("empty sequence 3a")
        publisher_result.publish(Odometry())
        rgb = cv2.resize(rgb, (0, 0), fx=output_scale, fy=output_scale)
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        publisher_image_gate.publish(output_im)
        return

    maximum_id_3a = np.argmax(vote_ms_3a)

    # find closest original cluster
    x_diff = x_ms_1 - x_ms_3a[maximum_id_3a]
    y_diff = y_ms_1 - y_ms_3a[maximum_id_3a]
    diff = x_diff * x_diff + y_diff * y_diff
    # temp_vote = vote_ms_1.copy()
    # for index in range(len(temp_vote)):
    #     if diff[index] > relocate_factor * relocate_factor * dist_thresh * dist_thresh:
    #         temp_vote[index] = 0
    # close_id_3a = np.argmax(temp_vote)
    close_id_3a = np.argmin(diff)

    corner_points[3, :] = [x_ms_1[close_id_3a], y_ms_1[close_id_3a]]

    # cv2.circle(rgb, (int(corner_points[3, 0]), int(corner_points[3, 1])), dist_thresh, (0, 255, 255), 2)
    # cv2.circle(rgb, (int(corner_points[3, 0]), int(corner_points[3, 1])), 2, (0, 255, 255), 2)

    # find all lines going from 2.1 to 3a
    # because in the end, algorithm will not use the found corners but the found lines
    close_id_3a = close_id_3a + 1  # to equal cluster number
    lines_cluster_3a_long = np.where(clusters_1 == close_id_3a)[0]
    lines_cluster_3a_short1 = lines_cluster_3a_long[lines_cluster_3a_long < len(lines)]
    lines_cluster_3a_short2 = lines_cluster_3a_long[lines_cluster_3a_long >= len(lines)] - len(lines)
    lines_cluster_3a = np.concatenate((lines_cluster_3a_short1, lines_cluster_3a_short2))

    # cluster b
    to_cluster1 = end[:, to_cluster_id1_b]
    to_cluster2 = start[:, to_cluster_id2_b]
    to_cluster = np.concatenate((to_cluster1, to_cluster2), axis=1)

    votes_3b = np.concatenate((votes[to_cluster_id1_b], votes[to_cluster_id2_b]))
    #
    # for i in range(np.shape(to_cluster)[1]):
    #     cv2.circle(rgb, (int(to_cluster[0][i]), int(to_cluster[1][i])), 2, (255, 0, 0), 2)

    clusters_3b = np.zeros(np.shape(to_cluster)[1])
    cluster = 0
    x_ms_3b = np.array([])
    y_ms_3b = np.array([])
    vote_ms_3b = np.array([])

    while not clusters_3b.all():
        cluster = cluster + 1
        s = np.where(clusters_3b == 0)[0][0]
        x_m = to_cluster[0][s]
        y_m = to_cluster[1][s]
        vote_m = votes_3b[s]
        clusters_3b[s] = cluster

        for idx in np.where(clusters_3b == 0)[0]:
            x = to_cluster[0][idx]
            y = to_cluster[1][idx]
            vote = votes_3b[idx]
            distance = math.sqrt((x - x_m) ** 2 + (y - y_m) ** 2)
            # print distance
            if distance < dist_thresh:
                x_m = (x_m * vote_m + x * vote) / (vote_m + vote)
                y_m = (y_m * vote_m + y * vote) / (vote_m + vote)
                vote_m = vote_m + vote
                clusters_3b[idx] = cluster

        x_ms_3b = np.append(x_ms_3b, x_m)
        y_ms_3b = np.append(y_ms_3b, y_m)
        vote_ms_3b = np.append(vote_ms_3b, vote_m)

    if cluster == 0:
        rospy.loginfo("empty sequence 3b")
        publisher_result.publish(Odometry())
        rgb = cv2.resize(rgb, (0, 0), fx=output_scale, fy=output_scale)
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        publisher_image_gate.publish(output_im)
        return

    maximum_id_3b = np.argmax(vote_ms_3b)

    # find closest original cluster
    x_diff = x_ms_1 - x_ms_3b[maximum_id_3b]
    y_diff = y_ms_1 - y_ms_3b[maximum_id_3b]
    diff = x_diff * x_diff + y_diff * y_diff
    # temp_vote = vote_ms_1.copy()
    # for index in range(len(temp_vote)):
    #     if diff[index] > relocate_factor * relocate_factor * dist_thresh * dist_thresh:
    #         temp_vote[index] = 0
    # close_id_3b = np.argmax(temp_vote)
    close_id_3b = np.argmin(diff)

    corner_points[4, :] = [x_ms_1[close_id_3b], y_ms_1[close_id_3b]]

    # cv2.circle(rgb, (int(corner_points[4, 0]), int(corner_points[4, 1])), dist_thresh, (0, 0, 255), 2)
    # cv2.circle(rgb, (int(corner_points[4, 0]), int(corner_points[4, 1])), 2, (0, 0, 255), 2)

    # find all lines going from 2.2 to 3b
    # because in the end, algorithm will not use the found corners but the found lines
    close_id_3b = close_id_3b + 1  # to equal cluster number
    lines_cluster_3b_long = np.where(clusters_1 == close_id_3b)[0]
    lines_cluster_3b_short1 = lines_cluster_3b_long[lines_cluster_3b_long < len(lines)]
    lines_cluster_3b_short2 = lines_cluster_3b_long[lines_cluster_3b_long >= len(lines)] - len(lines)
    lines_cluster_3b = np.concatenate((lines_cluster_3b_short1, lines_cluster_3b_short2))

    # define all lines that were found by algorithm respresenting all 4 sides of gate
    # bascially by intersecting the found line bunches with each other
    lines1 = lines_cluster_1[np.in1d(lines_cluster_1, lines_cluster_2a)]
    lines2 = lines_cluster_1[np.in1d(lines_cluster_1, lines_cluster_2b)]
    lines3 = lines_cluster_2a[np.in1d(lines_cluster_2a, lines_cluster_3a)]
    lines4 = lines_cluster_2b[np.in1d(lines_cluster_2b, lines_cluster_3b)]

    # draw lines
    # for line in lines1:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (0, 255, 0), 2)
    # for line in lines2:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 0, 0), 2)
    # for line in lines3:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 0, 255), 2)
    # for line in lines4:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 255, 0), 2)

    # make sure 4 line buches have been found, otherwise exit (if jungle special detection is not active)
    if not (lines1.any() and lines2.any() and lines3.any() and lines4.any()):
        rospy.loginfo("not four lines")
        publisher_result.publish(Odometry())
        rgb = cv2.resize(rgb, (0, 0), fx=output_scale, fy=output_scale)
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        publisher_image_gate.publish(output_im)
        return

    # intersect the relevant line bunches with each other to find intersection location. These are much more accurate
    # than the corners that have been found so far because these corners are only random start and end points of
    # lines and not their actual intersection
    (x1, y1) = isect_lines_bundle(lines1, lines2, start, end)
    (x2, y2) = isect_lines_bundle(lines1, lines3, start, end)
    (x3, y3) = isect_lines_bundle(lines2, lines4, start, end)
    (x4, y4) = isect_lines_bundle(lines3, lines4, start, end)

    # draw all these intersection points
    cv2.circle(rgb, (int(x1), int(y1)), dist_thresh, (0, 255, 255), 2)
    cv2.circle(rgb, (int(x2), int(y2)), dist_thresh, (0, 255, 255), 2)
    cv2.circle(rgb, (int(x3), int(y3)), dist_thresh, (0, 255, 255), 2)
    cv2.circle(rgb, (int(x4), int(y4)), dist_thresh, (0, 255, 255), 2)
    cv2.circle(rgb, (int(x1), int(y1)), 3, (0, 255, 255), 2)
    cv2.circle(rgb, (int(x2), int(y2)), 3, (0, 255, 255), 2)
    cv2.circle(rgb, (int(x3), int(y3)), 3, (0, 255, 255), 2)
    cv2.circle(rgb, (int(x4), int(y4)), 3, (0, 255, 255), 2)

    # redefine corner points
    corner_points = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])

    # Assume no lens distortion
    dist_coeffs = np.zeros((4, 1))

    # apply gate size
    square_side = gate_size

    # define 3D model points.
    model_points = np.array([
        (+square_side / 2, +square_side / 2, 0.0),
        (+square_side / 2, -square_side / 2, 0.0),
        (-square_side / 2, +square_side / 2, 0.0),
        (-square_side / 2, -square_side / 2, 0.0)])

    # solve Pnp algorithm
    (success, rvec, tvec) = cv2.solvePnP(model_points, corner_points, camera_matrix,
                                         dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

    # reduce the dimensions of the result
    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)
    # print "Rotation Vector:\n {0}".format(rvec)
    # print "Translation Vector:\n {0}".format(tvec)

    # publish results to gate detection message
    msg = Odometry()
    msg.twist.twist.linear.x = tvec[0]
    msg.twist.twist.linear.y = tvec[1]
    msg.twist.twist.linear.z = tvec[2]
    msg.twist.twist.angular.x = rvec[0]
    msg.twist.twist.angular.y = rvec[1]
    msg.twist.twist.angular.z = rvec[2]
    msg.pose.pose = this_pose
    publisher_result.publish(msg)
    rospy.loginfo("detected gate")

    rospy.loginfo("corner_points")
    rospy.loginfo(corner_points)

    # calculate a line that sticks out of the gate plane
    (center_point_2D_base, _) = cv2.projectPoints(np.array([(.0, .0, 0)]), rvec, tvec, camera_matrix, dist_coeffs)
    (center_point_2D_back, _) = cv2.projectPoints(np.array([(.0, .0, square_side)]), rvec, tvec, camera_matrix,
                                                  dist_coeffs)
    (center_point_2D_frnt, _) = cv2.projectPoints(np.array([(.0, .0, -square_side)]), rvec, tvec, camera_matrix,
                                                  dist_coeffs)

    # calculate the points that need to be drawn (backwards PnP basically)
    p1 = (int(center_point_2D_back[0][0][0]), int(center_point_2D_back[0][0][1]))
    p2 = (int(center_point_2D_frnt[0][0][0]), int(center_point_2D_frnt[0][0][1]))
    p3 = (int(center_point_2D_base[0][0][0]), int(center_point_2D_base[0][0][1]))

    # if these points make sense, draw them (check is turned off, was required earlier)
    if True or max(p1) < 10000 and max(p2) < 10000 and min(p1) > 0 and min(p2) > 0:
        cv2.line(rgb, p1, p3, (0, 255, 255), 10)
        cv2.line(rgb, p2, p3, (0, 255, 255), 10)
    # draw the center of the gate
    if True or max(p3) < 10000 and min(p3) > 0:
        cv2.circle(rgb, p3, 10, (255, 0, 0), -1)

    # publish image (no error)
    rgb = cv2.resize(rgb, (0, 0), fx=output_scale, fy=output_scale)
    output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
    publisher_image_gate.publish(output_im)

    # plt.pause(0.0001) # pause required so plotting with pyplt works
    # time.sleep(3)


def zed_pose_callback(data):
    # update odometry
    global latest_zed_pose
    latest_zed_pose = data.pose.pose


def bebop_pose_callback(data):
    # update odometry
    global latest_bebop_pose
    latest_bebop_pose = data.pose.pose


def camera_info_update(data):
    # update camera matrix
    global camera_matrix
    camera_matrix = np.resize(np.asarray(data.K), [3, 3])


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    # initialize node
    rospy.init_node('gate_detection', anonymous=False)

    # variables
    camera_matrix = None  # np.array([[669.86121, 0.0, 629.62378], [0.0, 669.86121, 384.62851], [0.0, 0.0, 1.0]])
    latest_zed_pose = None  # latest odometry
    latest_bebop_pose = None  # latest odometry
    gate_size = 1.4                             # initial gate size
    output_scale = 0.1                          # factor to scale all images with before broadcasting to save bandwidth
    orange_low = np.array([80, 150, 90])       # some original orange values
    orange_high = np.array([140, 255, 255])

    # initialize bridge to transform ROS images to CV matrices
    bridge = CvBridge()

    # subscribers
    rospy.Subscriber("/zed/left/camera_info", CameraInfo, camera_info_update)
    rospy.Subscriber("/zed/left/image_rect_color", Image, stereo_callback)
    rospy.Subscriber("/zed/odom", Odometry, zed_pose_callback)
    rospy.Subscriber("/bebop/odom", Odometry, bebop_pose_callback)

    # publishers
    publisher_image_threshold_orange = rospy.Publisher("/auto/gate_detection_threshold_orange", Image, queue_size=1)
    publisher_image_gate = rospy.Publisher("/auto/gate_detection_gate", Image, queue_size=1)
    publisher_result = rospy.Publisher("/auto/gate_detection_Odom", Odometry, queue_size=1)

    rospy.loginfo("running")
    rospy.spin()

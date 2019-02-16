#!/usr/bin/env python

# Script developed by Vincenz Frenzel

import rospy
import signal
import sys
from std_msgs.msg import Bool, Int32, Empty
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import Image


def signal_handler(_, __):
    sys.exit(0)


def image_callback(data):



if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    # initialize node
    rospy.init_node('gate_detection', anonymous=False)

    gate_size = 1.4                             # initial gate size
    output_scale = 0.1                          # factor to scale all images with before broadcasting to save bandwidth
    orange_low = np.array([100, 130, 50])       # some original orange values
    orange_high = np.array([130, 255, 255])
    # a factor currently not used for finding nearest cluster but allowing relocation if further clusters are much
    # stonger
    # relocate_factor = 1.5

    # initialize bridge to transform ROS images to CV matrices
    bridge = CvBridge()

    # subscribers
    rospy.Subscriber("/zed/rgb/camera_info", CameraInfo, camera_info_update)
    rospy.Subscriber("/zed/left/image_rect_color/compressed", Image, image_callback)

    # publishers
    publisher_image_gate = rospy.Publisher("/auto/gate_detection_gate", CompressedImage, queue_size=1)
    publisher_result = rospy.Publisher("/auto/gate_detection_result", Gate_Detection_Msg, queue_size=1)

    rospy.loginfo("running")
    rospy.spin()






# function[HSV, BW, E, HO, P_t, startend, coords, features_startend, features_coords, pts] = corners(RGB, threshold)
# pts = zeros(4, 2);
# features_startend = zeros(1, 4);
# features_coords = zeros(1, 2);
#
# % % color
# transform
# HSV = rgb2hsv(RGB);
#
# % % thresholding
# h = HSV(:,:, 1);
# s = HSV(:,:, 2);
# v = HSV(:,:, 3);
#
# if threshold(1, 1) < threshold(1, 2)
#     BW = ones(size(h));
#     BW(h < threshold(1, 1)) = 0;
#     BW(h > threshold(1, 2)) = 0;
# else
#     BW = zeros(size(h));
#     BW(h > threshold(1, 1)) = 1;
#     BW(h < threshold(1, 2)) = 1;
# end
# BW(s < threshold(2, 1)) = 0;
# BW(s > threshold(2, 2)) = 0;
# BW(v < threshold(3, 1)) = 0;
# BW(v > threshold(3, 2)) = 0;
# BW_gaus = imgaussfilt(double(BW), 1);
#
# % % edge
# detection
# E = edge(BW, 'roberts');
# E_gaus = imgaussfilt(double(E), 1);
#
# % % Hough
# line
# Transform
# RhoR = 5;
# [H, theta, rho] = hough(E, 'RhoResolution', RhoR, 'Theta', -90:0.5: 89.99);
# D = sqrt((size(E, 1) - 1) ^ 2 + (size(E, 2) - 1) ^ 2);
# nrho = 2 * (ceil(D / RhoR)) + 1;
# ntheta = length(theta);
# HO = zeros(nrho, ntheta);
# HO = H / max(H(:));
#
# % % Hough
# Peaks
# P = houghpeaks(H, 20, 'Threshold', 0.6 * max(H(:)), 'NHoodSize', [7 15]);
# P_t = P(:, [2 1]);
#
# % % Hough
# Lines
# lines = houghlines(BW_gaus, theta, rho, P, 'FillGap', 20, 'MinLength', 50);
# startend = zeros(length(lines), 4);
# for i = 1:length(lines)
# line = lines(i);
# startend(i,:) = [line.point1 line.point2];
# end
# coords = [startend(:, [1, 2]); startend(:, [3, 4])];
#
# dist = vecnorm(startend(:, [1, 2]) - startend(:, [3, 4]), 2, 2);
# hough_vote = diag(H(P(:, 1), P(:, 2)));
#
# if size(startend, 1) > 0
#     % % find
#     start
#     vert = 2000 - min(startend(:, [2, 4]), [], 2);
#     new_vote = dist. * vert;
#     [~, idx] = max(new_vote);
#
#     % % find
#     other
#     lines
#     features = zeros(200, 1);
#     features(1) = idx;
#
#     new = zeros(200, 1);
#     new(1) = idx;
#     new_amount = 1;
#     feature_amount = 1;
#     while new_amount > 0
#         old = new;
#         new = zeros(200, 1);
#         new_amount = 0;
#         for i = 1:length(old)
#         if old(i) > 0
#             % find
#             all
#             close
#             features
#             index = old(i);
#             % idx
#             linestart and end
#             p1 = startend(index, [1, 2]);
#             p2 = startend(index, [3, 4]);
#             % all
#             linestart and end
#             l1 = startend(:, [1, 2]);
#             l2 = startend(:, [3, 4]);
#             % distance
#             to
#             starts and to
#             ends
#             d1 = min(vecnorm(l1 - ones(size(l1, 1), 1) * p1, 2, 2), vecnorm(l1 - ones(size(l1, 1), 1) * p2, 2, 2));
#             d2 = min(vecnorm(l2 - ones(size(l2, 1), 1) * p1, 2, 2), vecnorm(l2 - ones(size(l2, 1), 1) * p2, 2, 2));
#             d = min(d1, d2);
#             all_idx = 1:length(d);
#             close_features = all_idx(d < 40);
#             add = close_features(~ismember(close_features, features));
#             add_amount = length(add);
#             if add_amount > 0
#                 new(new_amount + 1: new_amount + add_amount)=add;
#                 new_amount = new_amount + add_amount;
#                 features(feature_amount + 1: feature_amount + add_amount)=add;
#                 feature_amount = feature_amount + add_amount;
#             end
#         end
#     end
# end
#
# features_trim = features(features
# ~ = 0);
# features_startend = startend(features_trim,:);
# features_coords = [features_startend(:, [1, 2]); features_startend(:, [3, 4])];
#
# if size(features_trim, 1) > 3
#     % % create
#     bounding
#     box
#     bb = [min(features_coords), max(features_coords)];
#     corners_bb = [bb(1), bb(2);
#     bb(1), bb(4);
#     bb(3), bb(4);
#     bb(3), bb(2)];
#
#     % % cluster
#     points
#     dist_b1 = vecnorm(features_coords - ones(size(features_coords, 1), 1) * corners_bb(1,:), 2, 2);
#     dist_b2 = vecnorm(features_coords - ones(size(features_coords, 1), 1) * corners_bb(2,:), 2, 2);
#     dist_b3 = vecnorm(features_coords - ones(size(features_coords, 1), 1) * corners_bb(3,:), 2, 2);
#     dist_b4 = vecnorm(features_coords - ones(size(features_coords, 1), 1) * corners_bb(4,:), 2, 2);
#     [~, idx] = min([dist_b1, dist_b2, dist_b3, dist_b4], [], 2);
#     idx_n = idx(1:size(idx, 1) / 2).*idx(size(idx, 1) / 2 + 1: end);
#
#     % % intersect
#     2 - 6, 6 - 12, 12 - 4, 4 - 2
#     startend2 = features_startend(idx_n == 2,:);
#     startend6 = features_startend(idx_n == 6,:);
#     startend12 = features_startend(idx_n == 12,:);
#     startend4 = features_startend(idx_n == 4,:);
#
#     if ~isempty(startend2) & & ~isempty(startend6) & & ~isempty(startend12) & & ~isempty(startend4)
#         [x1, y1] = isect_lines_bundle(startend4, startend2);
#         [x2, y2] = isect_lines_bundle(startend2, startend6);
#         [x3, y3] = isect_lines_bundle(startend6, startend12);
#         [x4, y4] = isect_lines_bundle(startend12, startend4);
#
#         pts = [x1, y1;
#         x2, y2;
#         x3, y3;
#         x4, y4];
#         end
#     end
# end
#
# function[x_m, y_m] = isect_lines_bundle(lines1, lines2)
# % start and end
# points
# of
# all
# lines
# % create
# matrices
# from lists to
#
# enable
# fast
# numpy
# operations
# x1 = lines1(:, 1) *ones(1, size(lines2, 1));
# y1 = lines1(:, 2) *ones(1, size(lines2, 1));
# x2 = lines1(:, 3) *ones(1, size(lines2, 1));
# y2 = lines1(:, 4) *ones(1, size(lines2, 1));
# x3 = (lines2(:, 1) * ones(1, size(lines1, 1)))';
# y3 = (lines2(:, 2) * ones(1, size(lines1, 1)))';
# x4 = (lines2(:, 3) * ones(1, size(lines1, 1)))';
# y4 = (lines2(:, 4) * ones(1, size(lines1, 1)))';
#
# % basic
# idea: g1 = p1 + t * (p2 - p1)
# % g2 = p3 + s * (p4 - p3) and solve
# for s
#     s = ((x2 - x1). * (y3 - y1) - (x3 - x1). * (y2 - y1)). / ((x4 - x3). * (y2 - y1) - (x2 - x1). * (y4 - y3));
#
# % calculate
# mean
# of
# all
# intersection
# not taking
# into
# account
# invalid
# entries
# x = x3 + s. * (x4 - x3);
# y = y3 + s. * (y4 - y3);
# x_m = mean(x(:));
# y_m = mean(y(:));
# return
#

close all
clear
clc

dist = 3;
height = 2;

gates_preflight = [dist,0,height,deg2rad(090);
         0,dist,height,deg2rad(180);
        -dist,0,height,deg2rad(270);
        0,-dist,height,deg2rad(360)];
    
states_num = size(gates_preflight,1);

splines = zeros(3,4,size(gates,1));
splines(:,:,1) = calc_spline(gates(4,:),gates(1,:));
splines(:,:,2) = calc_spline(gates(1,:),gates(2,:));
splines(:,:,3) = calc_spline(gates(2,:),gates(3,:));
splines(:,:,4) = calc_spline(gates(3,:),gates(4,:));

%% drone
I = [1 0 0;
    0 1 0;
    0 0 1];
m = 2;
g = 9.81;
max_theta_X = (8)*pi/180; % radians
max_theta_Y = (8)*pi/180; % radians
max_v_Z = 1.0; % m/s
max_rot_Z = (60)*pi/180; % 13 deg... rad/s

Initial_pos = [0 -3 2];
Initial_vel_b =  [0.2 -0.2 0];
Initial_ori = [0 0 pi/4];

% 
% load('camer_params1000x1600.mat')
% IntrinsicMatrix = cameraParams.IntrinsicMatrix;
% FocalLength = cameraParams.FocalLength;
% PrincipalPoint = cameraParams.PrincipalPoint;

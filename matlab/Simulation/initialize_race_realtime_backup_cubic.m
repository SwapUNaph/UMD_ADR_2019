close all
clear
clc

load('PlotBus.mat','PlotBus');
gate_size = 1.4;
FOV = 1.05;
init_std_dev = [1.0*ones(1,3),deg2rad(30)];

dist = 3;
height = 2;
flightplan = [dist,0,height,deg2rad(090);
         0,dist,height,deg2rad(180);
        -dist,0,height,deg2rad(270);
        0,-dist,height,deg2rad(360)];
    
dist = 2.5;
gates = [dist,0,height,deg2rad(110);
         0,dist,height,deg2rad(180);
        -dist,0,height,deg2rad(270);
        0,-dist,height,deg2rad(360)];
    
states_num = size(flightplan,1);

%% drone
I = [1 0 0;
    0 1 0;
    0 0 1];
m = 2;
g = 9.81;

max_vel = 1.5;
max_theta_X = 75*pi/180; % radians
max_theta_Y = 75*pi/180; % radians
max_v_Z = 2*max_vel; % m/s
max_rot_Z = max_vel*(60)*pi/180; % 13 deg... rad/s

Initial_pos = [0 -3 2];
Initial_vel_b = max_vel/sqrt(2)*[1 -1 0];
Initial_vel_e = max_vel*[1,0,0];
Initial_ori = [0 0 pi/4];
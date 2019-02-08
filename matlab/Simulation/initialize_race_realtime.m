close all
clear
clc

load('Busses.mat');
gate_size = 1.4;
FOV = 1.05;
init_std_dev = [1.0*ones(1,3),deg2rad(30)];

dist = 3;
height = 2;
flightplan = [dist,0,height,deg2rad(090);
         0,dist,height,deg2rad(160);
        -dist,dist/2,height,deg2rad(345)-2*pi;
        0,-dist,height,deg2rad(340)-2*pi];
    
dist = 2.5;
gates = [dist,0,height,deg2rad(110);
         0,dist,height,deg2rad(160);
        -dist+0.5,dist/2,height,deg2rad(345)-2*pi;
        0,-dist+1,height,deg2rad(340)-2*pi];
    
states_num = size(flightplan,1);

WP1 = flightplan(end,:);
WP2 = flightplan(1,:);
hdg0 = WP1(4);
hdg3 = WP2(4);
P0 = WP1(1:3);
P3 = WP2(1:3);
m=1;
n=3;
P1 = [P0(1)+m*cos(hdg0), P0(2)+m*sin(hdg0), P3(3)];
P2 = P3 - n*[cos(hdg3) sin(hdg3) 0];

spline_init = [P0;P1;P2;P3];

std_dev_factor = 1.0;
std_dev_switch = std_dev_factor/5;

%% drone
I = [1 0 0;
    0 1 0;
    0 0 1];
m = 2;
g = 9.81;

max_vel = 1.0;
max_theta_X = 75*pi/180; % radians
max_theta_Y = 75*pi/180; % radians
max_v_Z = 2*max_vel; % m/s
max_rot_Z = max_vel*(60)*pi/180; % 13 deg... rad/s

Initial_pos = [0 -3 2];
Initial_vel_b = max_vel/sqrt(2)*[1 -1 0];
Initial_vel_e = max_vel*[1,0,0];
Initial_ori = [0 0 pi/4];

%% create WP bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'list';
elems(1).Dimensions = size(flightplan); 
elems(2) = Simulink.BusElement;
elems(2).Name = 'idx';
elems(2).Dimensions = 1;
WP_all_bus = Simulink.Bus;
WP_all_bus.Elements = elems;
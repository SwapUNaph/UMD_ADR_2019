close all
clear
clc

load('Busses.mat');
gate_size = 1.4;
two_std_dev_init = [0.8,0.8,1.2,deg2rad(45)];
two_std_dev_thres = 0.2;

resolution = [1280 720];
VFOV = 1.05/1.2;
HFOV = VFOV*resolution(1)/resolution(2);

height = 1.0;
takeoff = [0, 0, height, deg2rad(0)];
path_to_track = [3.0,0,0,deg2rad(0)]; % start to first gate
flightplan = [0,-5,0,deg2rad(0); % end to first gate
                4,0,0,deg2rad(0); % first gate to 2nd gate
             1.5,3.2,0,deg2rad(90);
             -6,1.5,0,deg2rad(180)];
             

         
%% initialize map
WP_list_init = zeros(size(flightplan));
WP_list_init(1,1:3) = takeoff(1:3) + path_to_track(1:3);
WP_list_init(1,4) = path_to_track(4);
for i = 2:size(flightplan,1)
    WP_list_init(i,1:3) = WP_list_init(i-1,1:3) + flightplan(i,1:3);
    WP_list_init(i,4) = flightplan(i,4);
end

%% initialize spline
WP1 = takeoff;
WP2 = WP_list_init(1,:);
hdg0 = WP1(4);
hdg3 = WP2(4);
P0 = WP1(1:3);
P3 = WP2(1:3);
m=1;
n=3;
P1 = [P0(1)+m*cos(hdg0), P0(2)+m*sin(hdg0), P3(3)];
P2 = P3 - n*[cos(hdg3) sin(hdg3) 0];

spline_init = [P0;P1;P2;P3];

%% drone
g = 9.81;

max_vel = 1.0;
max_theta_X = 8*pi/180; % radians
max_theta_Y = 8*pi/180; % radians
max_v_Z = 1*max_vel; % m/s
max_rot_Z = (13)*pi/180; % 13 deg... rad/s

%% create WP bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'list';
elems(1).Dimensions = size(flightplan); 
elems(2) = Simulink.BusElement;
elems(2).Name = 'idx';
elems(2).Dimensions = 1;
WP_all_bus = Simulink.Bus;
WP_all_bus.Elements = elems;
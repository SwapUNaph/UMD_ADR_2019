close all

load('Busses.mat');
mass = 0.5;
g = 9.81;

gate_size = 1.4;
two_std_dev_init = [1,1,0.5,deg2rad(30)];
two_std_dev_thres = 0.3;

resolution = 0.5*[1280 720];
VFOV = 1.05;
HFOV = VFOV*resolution(1)/resolution(2);

height = 2.3;
flightplan = [ 4.348, -0.278, height, deg2rad(  0);
              10.805, -0.278, height, deg2rad(  0);
              12.146, -3.775, height, deg2rad(-90);
               8.769, -6.876, height, deg2rad(180);
               2.593, -6.764, height, deg2rad(180);
               2.657, -3.652, height, deg2rad(  0);
               7.172, -3.648, height, deg2rad( 90);
               4.348, -0.278, height, deg2rad(180)];
takeoff = [0, 0, height, deg2rad(  0)];

%% center flightplan and takeoff
takeoff = takeoff - [0.5*min(flightplan(:,1:2))+0.5*max(flightplan(:,1:2)), 0, 0];
flightplan = flightplan - [0.5*min(flightplan(:,1:2))+0.5*max(flightplan(:,1:2)), 0, 0];
    
%% noise
% gate
std_dev_gate = [0.35 0.35 0.1 deg2rad(10)];

% state
std_dev_xyz = 0.06*ones(1,3);
std_dev_eul = 0.25*deg2rad(0.8)*ones(1,3);
std_dev_vel = 0.06*ones(1,3);

% physical
max_theta_X = 24*pi/180; % radians 75
max_force = mass*g*tan(max_theta_X);
max_moment = 10;
std_dev_F = 0.1*max_force*ones(1,3);
std_dev_M = 0.1*max_moment*[0, 0, 1];


%% init gates
pert = [normrnd(0,std_dev_gate(1),7,1), normrnd(0,std_dev_gate(2),7,1), normrnd(0,std_dev_gate(3),7,1), normrnd(0,std_dev_gate(4),7,1)];
pert = [pert; pert(1,:)];
gates = flightplan + pert;

%% initialize some spline
WP1 = takeoff;
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

%% drone
I = [1 0 0;
    0 1 0;
    0 0 1];

max_vel = 2.0;
max_theta_X = 24*pi/180; % radians 75
max_theta_Y = 24*pi/180; % radians 75
max_v_Z = 3*max_vel; % m/s
max_rot_Z = (180)*pi/180; % 60 deg... rad/s

Initial_pos = takeoff(1:3);
Initial_vel_b = [0.1 0 0];
Initial_vel_e = [0.1*cos(takeoff(4)),0.1*sin(takeoff(4)),0];
Initial_ori = [0 0 takeoff(4)];

%% create WP bus
elems(1) = Simulink.BusElement;
elems(1).Name = 'list';
elems(1).Dimensions = size(flightplan); 
elems(2) = Simulink.BusElement;
elems(2).Name = 'idx';
elems(2).Dimensions = 1;
WP_all_bus = Simulink.Bus;
WP_all_bus.Elements = elems;
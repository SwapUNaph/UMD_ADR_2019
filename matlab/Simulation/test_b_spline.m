clear
close all
clc

dist = 3;
height = 2;
flightplan = [dist,0,height,deg2rad(090);
         0,dist,height,deg2rad(180);
        -dist,0,height,deg2rad(270);
        0,-dist,height,deg2rad(360)];
    
tau = [0,2,5];

u = linspace(tau(1),tau(2),100);

N_10t = 1;
N_20t = 1;
N_11t = (u-tau(1))/(tau(2)-tau(1))*N_10t+(tau(3)-u)/(tau(3)-tau(2))*N_20t;
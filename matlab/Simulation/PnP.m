close all
clc

drone_ori = [0.9923, -0.08767, -0.08767, 1.579];
drone_pos = [2.73, -1.53, 2];

plot3(drone_pos(1),drone_pos(2),drone_pos(3),'xk')
hold on
grid on
axis equal

dRs = axang2rotm(drone_ori);
gRs = eul2rotm([-pi/2, 0, pi/2]);
sRg = gRs^-1;
dRg = dRs*sRg;

dx=dRg*[1,0,0]';
dy=dRg*[0,1,0]';
dz=dRg*[0,0,1]';

% plot drone ori (front,left,up)
plot3(drone_pos(1)+[0, dx(1)],drone_pos(2)+[0, dx(2)],drone_pos(3)+[0 dx(3)],'r-')
plot3(drone_pos(1)+[0, dy(1)],drone_pos(2)+[0, dy(2)],drone_pos(3)+[0 dy(3)],'g-')
plot3(drone_pos(1)+[0, dz(1)],drone_pos(2)+[0, dz(2)],drone_pos(3)+[0 dz(3)],'b-')

% plot gates
plot3(gates(:,1),gates(:,2),gates(:,3),'xr')

%% convert gate pos
tvec = [0.006171, 0.007421, 1.484]; % right down front
% rx = +tvec(3)*dx;
% ry = -tvec(1)*dy;
% rz = -tvec(2)*dz;
% rel = rx+ry+rz;
dRc = eul2rotm([pi/2,-pi/2,0],'ZYX');
cRd = dRc^-1;
cRg = dRg*cRd;
rel= cRg*tvec';
gate_pos = drone_pos + rel';
plot3(gate_pos(1),gate_pos(2),gate_pos(3),'ro')

% rel = -dRs*tvec';
% gate_pos = drone_pos + rel';
% plot3(gate_pos(1),gate_pos(2),gate_pos(3),'go')

%% convert gate orientation
rmat = [-0.0004401, 1, -8.02e-05;
    0.9838, 0.0004473, 0.1793;
    0.1793, 9.117e-18, -0.9838];

wRc = rmat;
one_more = eul2rotm([-pi/2,pi/2,0]);
another_one = 1;%*eul2rotm([0,pi/2,0]);
wRs = another_one*one_more*wRc*cRg^-1;

gx=wRs*[1,0,0]';
gy=wRs*[0,1,0]';
gz=wRs*[0,0,1]';

gate_ori_global = (rotm2axang(wRs))

% plot gate ori
plot3(gate_pos(1)+[0, gx(1)],gate_pos(2)+[0, gx(2)],gate_pos(3)+[0 gx(3)],'r-')
plot3(gate_pos(1)+[0, gy(1)],gate_pos(2)+[0, gy(2)],gate_pos(3)+[0 gy(3)],'g-')
plot3(gate_pos(1)+[0, gz(1)],gate_pos(2)+[0, gz(2)],gate_pos(3)+[0 gz(3)],'b-')





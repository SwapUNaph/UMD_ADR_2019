clear
close all
clc
figure('units','normalized','outerposition',[0 0 1 1])

syms n_s m_s
span=0.5:0.5:1.5;
d = (0:0.01:1)';

% k0=2/3*abs((P1(1)-P0(1))*(P2(2)-2*P1(2)+P0(2))-(P1(2)-P0(2))*(P2(1)-2*P1(1)+P0(1)))/((P1(1)-P0(1))^2+(P1(2)-P0(2))^2)^1.5
% k1=2/3*abs((P3(1)-P2(1))*(P3(2)-2*P2(2)+P1(2))-(P3(2)-P2(2))*(P3(1)-2*P2(1)+P1(1)))/((P3(1)-P2(1))^2+(P3(2)-P2(2))^2)^1.5

%% WP
subplot(2,2,1)

dist = 10;
height = 2;
flightplan = [dist,0,height,deg2rad(090);
         sqrt(dist*dist/2),sqrt(dist*dist/2),height,deg2rad(135);
        -dist,0,height,deg2rad(270);
        0,-dist,height,deg2rad(360)];
    
WP1 = flightplan(1,:);
WP2 = flightplan(2,:);

P0 = WP1(1:3);
P3 = WP2(1:3);
hdg0 = WP1(4);
hdg3 = WP2(4);

r = round((P0(1)-P3(1))/(sin(hdg0)-sin(hdg3)),2);
M = P0 + r * [-sin(hdg0), cos(hdg0), 0];
plot3(M(1),M(2),M(3),'xr')
hold on

c0 = 1/r;
c1 = 1/r;
dp = hdg3-hdg0;

% e(1) = c0 == (2*abs(r*cos(dp) + n_s*sin(dp) - r))/(3*m_s^2);
% e(2) = c1 == (2*abs(r*cos(dp) - m_s*sin(dp) + r))/(3*n_s^2);
% res = vpasolve(e,[m_s n_s],[2.6511477349,2.6511477349]);
% m = res.m_s;
% n = res.n_s;
% m = 2.6511477349;
% n=m;
e = c0 == 2*abs(r*cos(dp)+m_s*sin(dp)-r)/(3*m_s^2);
m = vpasolve(e,2);
n = m;

P1 = P0 + [m*cos(hdg0), m*sin(hdg0), 1*(P3(3)-P0(3))];
P2 = P3 - [n*cos(hdg3), n*sin(hdg3), 0              ];

current_spline = [P0; P1; P2; P3];

% spline
% spline_pos_quad = (1-d)*u.*((1-d)*P0+d*P1)+d*u.*((1-d)*P1 + d*P2);
spline_pos_cubic = (1-d).^3*P0 + 3*(1-d).^2.*d*P1 + 3*(1-d).*d.^2*P2 + d.^3*P3;

plot3(spline_pos_cubic(:,1),spline_pos_cubic(:,2),spline_pos_cubic(:,3),'-r','LineWidth',2); %5
plot3([P1(1) P2(1)],[P1(2),P2(2)],[P1(3),P2(3)],'xb'); %5
 

% for n=span*r
%     for m = span*r
%         P1 = P0 + [m*cos(hdg0), m*sin(hdg0), 1*(P3(3)-P0(3))];
%         P2 = P3 - [n*cos(hdg3), n*sin(hdg3), 0              ];
% 
%         current_spline = [P0; P1; P2; P3];
% 
%         % spline
%         % spline_pos_quad = (1-d)*u.*((1-d)*P0+d*P1)+d*u.*((1-d)*P1 + d*P2);
%         spline_pos_cubic = (1-d).^3*P0 + 3*(1-d).^2.*d*P1 + 3*(1-d).*d.^2*P2 + d.^3*P3;
% 
%         plot3(spline_pos_cubic(:,1),spline_pos_cubic(:,2),spline_pos_cubic(:,3),'-','LineWidth',2,'Color',[n/r-0.5,m/r-0.5,0],'DisplayName',['m' num2str(m/r) 'n' num2str(n/r)]); %5
%         plot3([P1(1) P2(1)],[P1(2),P2(2)],[P1(3),P2(3)],'x','Color',[n/r-0.5,m/r-0.5,0.5],'DisplayName',['m' num2str(m/r) 'n' num2str(n/r)]); %5
% 
%     end
% end

t=d*(hdg3-hdg0)+hdg0;
plot3(r*sin(t),-r*cos(t),0*t,'--k')

grid on
axis equal
view(0,90)
legend('Location','southwestoutside')



%% WP
subplot(2,2,2)

dist = 10;
height = 2;
flightplan = [dist,0,height,deg2rad(090);
         0,dist,height,deg2rad(180)];
    
WP1 = flightplan(1,:);
WP2 = flightplan(2,:);

P0 = WP1(1:3);
P3 = WP2(1:3);
hdg0 = WP1(4);
hdg3 = WP2(4);

r = round((P0(1)-P3(1))/(sin(hdg0)-sin(hdg3)),2);
M = P0 + r * [-sin(hdg0), cos(hdg0), 0];
plot3(M(1),M(2),M(3),'xr')
hold on

c0 = 1/r;
c1 = 1/r;
dp = hdg3-hdg0;

% e(1) = c0 == (2*abs(r*cos(dp) + n_s*sin(dp) - r))/(3*m_s^2);
% e(2) = c1 == (2*abs(r*cos(dp) - m_s*sin(dp) + r))/(3*n_s^2);
% res = vpasolve(e,[m_s n_s],[2.6511477349,2.6511477349]);
% m = res.m_s;
% n = res.n_s;
% m = 2.6511477349;
% n=m;
e = c0 == 2*abs(r*cos(dp)+m_s*sin(dp)-r)/(3*m_s^2);
m = vpasolve(e,r);
n = m;

P1 = P0 + [m*cos(hdg0), m*sin(hdg0), 1*(P3(3)-P0(3))];
P2 = P3 - [n*cos(hdg3), n*sin(hdg3), 0              ];

current_spline = [P0; P1; P2; P3];

% spline
% spline_pos_quad = (1-d)*u.*((1-d)*P0+d*P1)+d*u.*((1-d)*P1 + d*P2);
spline_pos_cubic = (1-d).^3*P0 + 3*(1-d).^2.*d*P1 + 3*(1-d).*d.^2*P2 + d.^3*P3;

plot3(spline_pos_cubic(:,1),spline_pos_cubic(:,2),spline_pos_cubic(:,3),'-r','LineWidth',2); %5
plot3([P1(1) P2(1)],[P1(2),P2(2)],[P1(3),P2(3)],'xb'); %5


t=d*(hdg3-hdg0)+hdg0;
plot3(r*sin(t),-r*cos(t),0*t,'--k')

grid on
axis equal
view(0,90)
xlabel('x')

%% WP
subplot(2,2,3)

dist = 10;
height = 2;
flightplan = [dist,0,height,deg2rad(090);
         -sqrt(dist*dist/2),sqrt(dist*dist/2),height,deg2rad(225)];
    
WP1 = flightplan(1,:);
WP2 = flightplan(2,:);

P0 = WP1(1:3);
P3 = WP2(1:3);
hdg0 = WP1(4);
hdg3 = WP2(4);

r = round((P0(1)-P3(1))/(sin(hdg0)-sin(hdg3)),2);
M = P0 + r * [-sin(hdg0), cos(hdg0), 0];
plot3(M(1),M(2),M(3),'xr')
hold on

c0 = 1/r;
c1 = 1/r;
dp = hdg3-hdg0;


% e(1) = c0 == (2*abs(r*cos(dp) + n_s*sin(dp) - r))/(3*m_s^2);
% e(2) = c1 == (2*abs(r*cos(dp) - m_s*sin(dp) + r))/(3*n_s^2);
% res = vpasolve(e,[m_s n_s],[2.6511477349,2.6511477349]);
% m = res.m_s;
% n = res.n_s;
% m = 2.6511477349;
% n=m;
e = c0 == 2*abs(r*cos(dp)+m_s*sin(dp)-r)/(3*m_s^2);
m = vpasolve(e,r);
n = m;

P1 = P0 + [m*cos(hdg0), m*sin(hdg0), 1*(P3(3)-P0(3))];
P2 = P3 - [n*cos(hdg3), n*sin(hdg3), 0              ];

current_spline = [P0; P1; P2; P3];

% spline
% spline_pos_quad = (1-d)*u.*((1-d)*P0+d*P1)+d*u.*((1-d)*P1 + d*P2);
spline_pos_cubic = (1-d).^3*P0 + 3*(1-d).^2.*d*P1 + 3*(1-d).*d.^2*P2 + d.^3*P3;

plot3(spline_pos_cubic(:,1),spline_pos_cubic(:,2),spline_pos_cubic(:,3),'-r','LineWidth',2); %5
plot3([P1(1) P2(1)],[P1(2),P2(2)],[P1(3),P2(3)],'xb'); %5


t=d*(hdg3-hdg0)+hdg0;
plot3(r*sin(t),-r*cos(t),0*t,'--k')

grid on
axis equal
view(0,90)

%% WP
subplot(2,2,4)

dist = 10;
height = 2;
flightplan = [dist,0,height,deg2rad(090);
        -sqrt(dist*dist/2),-sqrt(dist*dist/2),height,deg2rad(315)];
    
WP1 = flightplan(1,:);
WP2 = flightplan(2,:);

P0 = WP1(1:3);
P3 = WP2(1:3);
hdg0 = WP1(4);
hdg3 = WP2(4);

r = round((P0(1)-P3(1))/(sin(hdg0)-sin(hdg3)),2);
M = P0 + r * [-sin(hdg0), cos(hdg0), 0];
plot3(M(1),M(2),M(3),'xr')
hold on

c0 = 1/r;
c1 = 1/r;
dp = hdg3-hdg0;


% e(1) = c0 == (2*abs(r*cos(dp) + n_s*sin(dp) - r))/(3*m_s^2);
% e(2) = c1 == (2*abs(r*cos(dp) - m_s*sin(dp) + r))/(3*n_s^2);
% res = vpasolve(e,[m_s n_s],[2.6511477349,2.6511477349]);
% m = res.m_s;
% n = res.n_s;
% m = 2.6511477349;
% n=m;
e = c0 == 2*abs(r*cos(dp)+m_s*sin(dp)-r)/(3*m_s^2);
m = vpasolve(e,r);
n = m;

P1 = P0 + [m*cos(hdg0), m*sin(hdg0), 1*(P3(3)-P0(3))];
P2 = P3 - [n*cos(hdg3), n*sin(hdg3), 0              ];

current_spline = [P0; P1; P2; P3];

% spline
% spline_pos_quad = (1-d)*u.*((1-d)*P0+d*P1)+d*u.*((1-d)*P1 + d*P2);
spline_pos_cubic = (1-d).^3*P0 + 3*(1-d).^2.*d*P1 + 3*(1-d).*d.^2*P2 + d.^3*P3;

plot3(spline_pos_cubic(:,1),spline_pos_cubic(:,2),spline_pos_cubic(:,3),'-r','LineWidth',2); %5
plot3([P1(1) P2(1)],[P1(2),P2(2)],[P1(3),P2(3)],'xb'); %5


t=d*(hdg3-hdg0)+hdg0;
plot3(r*sin(t),-r*cos(t),0*t,'--k')

grid on
axis equal
view(0,90)
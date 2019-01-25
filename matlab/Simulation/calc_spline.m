function spline = calc_spline(WP1, WP2)
%CA Summary of this function goes here
%   Detailed explanation goes here
    
%% position
% B(d) = (1-d)[(1-d)P0+dP1]+d[(1-d)P1 + dP2]
% B'(d) = 2(1-d)(P1-P0) + 2d(P2-P1)

% B(0) = P0
% B(1) = P2

P0 = WP1(1:3);
P2 = WP2(1:3);

% g1: WP1(1:3) + m*[cos(WP1(4));sin(WP1(4));0]
% g2: WP2(1:3) - n*[cos(WP2(4));sin(WP2(4));0]

A = [cos(WP1(4)) cos(WP2(4));
     sin(WP1(4)) sin(WP2(4))];
b = -[WP1(1)-WP2(1);
    WP1(2)-WP2(2)];
r = A\b;
m = r(1);
P1 = WP1(1:3) + m*[cos(WP1(4)), sin(WP1(4)), 0];

spline = [P0; P1; P2];

%% heading
% B(d) = (1-d)[(1-d)P0+dP1]+d[(1-d)P1 + dP2]
% B'(d) = 2(1-d)(P1-P0) + 2d(P2-P1)

% B(0) = atan2(gates(2,2)-gates(1,2),gates(2,1)-gates(1,1))
% B(1) = gates(2,4)

P0 = atan2(WP2(2)-WP1(2),WP2(1)-WP1(1));
P2 = WP2(4);
unwr = unwrap([P0,P2]);
P0 = unwr(1);
P2 = unwr(2);

m = 0.68;
P1 = P0 + m*(P2-P0);

spline = [spline, [P0;P1;P2]];

end
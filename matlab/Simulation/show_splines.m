close all

figure('units','normalized','outerposition',[0 0 1 1])
hold on
axis equal

pos = gates(:,1:3);
or = gates(:,4);

plot_gate(gates(1,:))
plot_gate(gates(2,:))
plot_gate(gates(3,:))
plot_gate(gates(4,:))

plot_ground(dist*1.5)
axis([-1.3*dist,1.3*dist,-1.3*dist,1.3*dist,0,3])
view(28,28)

splines = zeros(3,4,size(gates,1));
splines(:,:,1) = calc_plot_spline(gates(4,:),gates(1,:));
splines(:,:,2) = calc_plot_spline(gates(1,:),gates(2,:));
splines(:,:,3) = calc_plot_spline(gates(2,:),gates(3,:));
splines(:,:,4) = calc_plot_spline(gates(3,:),gates(4,:));

%% calc heading spline



%% compare spline heading difference

d = (0:0.01:1)';

angle0_best = atan2(gates(2,2)-gates(1,2),gates(2,1)-gates(1,1));
angle1_best = atan2(gates(3,2)-gates(2,2),gates(3,1)-gates(2,1));
angle0_worst = gates(1,4);
angle1_worst = gates(2,4);
unwr = unwrap([angle0_best,angle1_best,angle0_worst,angle1_worst]);
angle0_best = unwr(1);
angle1_best = unwr(2);
angle0_worst = unwr(3);
angle1_worst = unwr(4);

P0s = (angle0_best+angle0_worst)/2;
P3s = (angle1_best+angle1_worst)/2;
unwr = unwrap([P0s,P3s]);
P0s = unwr(1);
P3s = unwr(2);

% cubic
%B(d) = (1−d).^3*P0s + 3*(1−d).^2*d*P1s + 3*(1−d)*d.^2*P2s + d.^3*P3s
%B'(d) = 3*(1-d).^2*(P1-P0) + 6*(1-d)d*(P2-P1) + 3d^2*(P3-P2)
%B'(0) = 3*(P1-P0)
%B'(1) = 3*(P3-P2)
% P1-P0 = P3-P2

P1s = P0s+1.3;
P2s = P3s-1.1;
unwr = unwrap([P0s,P1s,P2s,P3s]);
P0s = unwr(1);
P1s = unwr(2);
P2s = unwr(3);
P3s = unwr(4);

figure('units','normalized','outerposition',[0 0 1 1])
hold on

spline = (1-d).^3*P0s + 3*(1-d).^2.*d*P1s + 3*(1-d).*d.^2*P2s + d.^3*P3s;

plot(d,spline*180/pi,'-r')

%% min jerk
% looks really bad

%% precise

P0 = splines(1,:,2);
P1 = splines(2,:,2);
P2 = splines(3,:,2);
unit=ones(1,size(P0,2));
spline = (1-d)*unit.*((1-d)*P0+d*P1)+d*unit.*((1-d)*P1 + d*P2);
angle=atan2(gates(2,2)-spline(:,2),gates(2,1)-spline(:,1));
    
plot(d(1:end-1),angle(1:end-1)*180/pi,'x-k')


P0 = splines(1,:,2);
P1 = splines(2,:,2);
P2 = splines(3,:,2);
unit=ones(1,size(P0,2));
spline = (1-d)*unit.*((1-d)*P0+d*P1)+d*unit.*((1-d)*P1 + d*P2);
angle=atan2(gates(2,2)-spline(:,2),gates(2,1)-spline(:,1));
plot(d(1:end-1),angle(1:end-1)*180/pi,'x-k')
angle=atan2(gates(3,2)-spline(:,2),gates(3,1)-spline(:,1));
angle = unwrap(angle);
plot(d(1:end-1),angle(1:end-1)*180/pi,'x-b')

grid on

function plot_gate(WP)
    x = WP(1);
    y = WP(2);
    z = WP(3);
    o = WP(4);
    
    s = 1.4;
    
    xs = x*ones(5,1) + s/2*[sin(o);-sin(o);-sin(o);sin(o);sin(o)];
    ys = y*ones(5,1) + s/2*[cos(o);-cos(o);-cos(o);cos(o);cos(o)];
    zs = z*ones(5,1) + s/2*[-1; -1; +1; +1; -1];
    
    orange = [1,0.5,0];
    plot3(xs,ys,zs,'-','Color',orange,'LineWidth',4)
    
    quiver3(x,y,z,cos(o),sin(o),0,'Color','r','LineWidth',2,'MarkerSize',10)
end

function plot_ground(dist)
    [X,Y] = meshgrid(-dist:dist,-dist:dist);
    Z = zeros(size(X));
    surf(X,Y,Z,'EdgeColor','k','FaceColor',[0.5,0.5,0.5])

end












clear
close all
clc
figure('units','normalized','outerposition',[0 0 1 1])

syms n_s m_s
span=1:0.25:2.5;
d = (0:0.01:1)';

height = 2;
dist = [10; 10; 10; 10];
WP1s = [dist,0+0*dist,height+0*dist,deg2rad(090)+0*dist];
WP2s = [sqrt(dist(1)^2/2),sqrt(dist(1)^2/2),height,deg2rad(135);
    0,dist(2),height,deg2rad(180);
    -sqrt(dist(3)^2/2),sqrt(dist(3)^2/2),height,deg2rad(225);
    -sqrt(dist(4)^2/2),-sqrt(dist(4)^2/2),height,deg2rad(315)];
    
for i=1:4    
    subplot(2,2,i)

    WP1 = WP1s(i,:);
    WP2 = WP2s(i,:);

    P0 = WP1(1:3);
    P3 = WP2(1:3);
    hdg0 = WP1(4);
    hdg3 = WP2(4);

    r = round((P0(1)-P3(1))/(sin(hdg0)-sin(hdg3)),2);
    M = P0 + r * [-sin(hdg0), cos(hdg0), 0];
    plot3(M(1),M(2),M(3),'xr')
    hold on
    
    t=d*(hdg3-hdg0)+hdg0;
    plot3(r*sin(t),-r*cos(t),0*t,'--k')

    c0 = 1/r;
    c1 = 1/r;
    dp = hdg3-hdg0;

%     e = c0 == 2*abs(r*cos(dp)+m_s*sin(dp)-r)/(3*m_s^2);
%     m = double(vpasolve(e,2));
%     m=m(end)
    m=r/3*(-sin(dp)+sqrt(sin(dp)^2-6*cos(dp)+6));
    
    for n=span*m
        P1 = P0 + [m*cos(hdg0), m*sin(hdg0), 1*(P3(3)-P0(3))];
        P2 = P3 - [n*cos(hdg3), n*sin(hdg3), 0              ];

        % spline
        % spline_pos_quad = (1-d)*u.*((1-d)*P0+d*P1)+d*u.*((1-d)*P1 + d*P2);
        spline_pos_cubic = (1-d).^3*P0 + 3*(1-d).^2.*d*P1 + 3*(1-d).*d.^2*P2 + d.^3*P3;

        plot3(spline_pos_cubic(:,1),spline_pos_cubic(:,2),spline_pos_cubic(:,3),'-','LineWidth',2,'Color',[n/2/m-0.5,0.5,n/2/m-0.5],'DisplayName',['span' num2str(n/m)]); %5
        plot3([P1(1) P2(1)],[P1(2),P2(2)],[P1(3),P2(3)],'x','Color',[n/2/m-0.5,0.5,n/2/m-0.5],'DisplayName',['span' num2str(n/m)]); %5
    end

    grid on
    axis equal
    view(0,90)
    
end

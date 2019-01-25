close all
clc
figure('units','normalized','outerposition',[0 0 1 1])


d_num = (0:0.01:1)';
%%
amount = size(spline.Data,3);
for i=2:(amount-0)
    % B(d) = (1-d)^3*P0 + 3(1-d)^2*d*P1 + 3(1-d)*d^2*P2 + d^3*P3
    % B'(d) = 3(1-d)^2(P1-P0) + 6*(1-d)*d*(P2-P1) + 3d^2(P3-P2)

    P0_this = spline.Data(1,:,i);
    P1_this = spline.Data(2,:,i);
    P2_this = spline.Data(3,:,i);
    P3_this = spline.Data(4,:,i);
    P0_old = spline.Data(1,:,i-1);
    P1_old = spline.Data(2,:,i-1);
    P2_old = spline.Data(3,:,i-1);
    P3_old = spline.Data(4,:,i-1);
    
    subplot(1,2,1)
    spline_pos_cubic = (1-d_num).^3*P0_this + 3*(1-d_num).^2.*d_num*P1_this + 3*(1-d_num).*d_num.^2*P2_this + d_num.^3*P3_this;
    plot(-spline_pos_cubic(:,2),spline_pos_cubic(:,1),':k','LineWidth',2); %5

    axis equal
    axis([-5 5 -5 5])
    grid on
    
    subplot(1,2,2)
    spline_prime_cubic = 3*(1-d_num).^2*(P1_this-P0_this) + 6*(1-d_num).*d_num*(P2_this-P1_this) + 3*d_num.^2*(P3_this-P2_this);
    dist = sqrt(sum(spline_prime_cubic.^2,2));
    plot(d_num,dist,'-','Color',[0.5,i/amount,i/amount],'LineWidth',2); %5
    hold on
    axis([0 1 0 12])
    grid on
    
    dx0_this=P1_this(1)-P0_this(1);
    dx1_this=P2_this(1)-P1_this(1);
    dx2_this=P3_this(1)-P2_this(1);
    dy0_this=P1_this(2)-P0_this(2);
    dy1_this=P2_this(2)-P1_this(2);
    dy2_this=P3_this(2)-P2_this(2);
    dx0_old=P1_old(1)-P0_old(1);
    dx1_old=P2_old(1)-P1_old(1);
    dx2_old=P3_old(1)-P2_old(1);
    dy0_old=P1_old(2)-P0_old(2);
    dy1_old=P2_old(2)-P1_old(2);
    dy2_old=P3_old(2)-P2_old(2);
    
%     syms d_s
    fun_this = @(d_s) 3*sqrt((1-d_s).^4*(dx0_this^2+dy0_this^2)+4*d_s.^2.*(1-d_s).^2*(dx1_this^2+dy1_this^2)+d_s.^4*(dx2_this^2+dy2_this^2)+4*d_s.*(1-d_s).^3*(dx0_this*dx1_this+dy0_this*dy1_this)+2*d_s.^2.*(1-d_s).^2*(dx0_this*dx2_this+dy0_this*dy2_this)+4*d_s.^3.*(1-d_s)*(dx1_this*dx2_this+dy1_this*dy2_this));
    fun_old = @(d_s) 3*sqrt((1-d_s).^4*(dx0_old^2+dy0_old^2)+4*d_s.^2.*(1-d_s).^2*(dx1_old^2+dy1_old^2)+d_s.^4*(dx2_old^2+dy2_old^2)+4*d_s.*(1-d_s).^3*(dx0_old*dx1_old+dy0_old*dy1_old)+2*d_s.^2.*(1-d_s).^2*(dx0_old*dx2_old+dy0_old*dy2_old)+4*d_s.^3.*(1-d_s)*(dx1_old*dx2_old+dy1_old*dy2_old));
%     fnc=3*sqrt((1-d_s)^4*(dx0^2+dy0^2)+4*d_s^2*(1-d_s)^2*(dx1^2+dy1^2)+d_s^4*(dx2^2+dy2^2)+4*d_s*(1-d_s)^3*(dx0*dx1+dy0*dy1)+2*d_s^2*(1-d_s)^2*(dx0*dx2+dy0*dy2)+4*d_s^3*(1-d_s)*(dx1*dx2+dy1*dy2));
%     fnc_num = subs(fnc,d_s,d_num);
    
    d_this = d.Data(i);
    d_old = d.Data(i-1);
    
    dist_this = integral(fun_this,0,d_old);
    dist_old = integral(fun_old,0,d_old);
    
    d_new = d_old + (dist_this-dist_old)*fun_this(d_old)
    pause(0.1)
    

    
end
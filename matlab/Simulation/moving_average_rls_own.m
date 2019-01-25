close all
clc
clear
figure('units','normalized','outerposition',[0 0 1 1])

measurements = [];
r1=randn(20,1)/1.5-4;
r2=randn(20,1)/3.5-4;
r3=randn(20,1)/5-3.7;
r4=randn(20,1)/12-4;
measurements=[measurements;-4;-5;r1;-5;-1;-6;-1;r2;-6;-1.5;-1;-5.5;r3;-6.5;-0;-1;-0;r4;-0;-6;-5];

p=0;
lambda=0.95;
delta=5;
w=zeros(size(measurements));
w(1)=-4;
d=0;
P = zeros(size(measurements));
P(1)=delta*1;

for i=2:length(measurements)
    meas = measurements(i)
    
    alpha = d-meas*w(i-1)
    g = P(i-1)*meas*(lambda+meas*P(i-1)*meas)^-1
    P(i)=lambda^-1*P(i-1)-g*meas*lambda^-1*P(i-1);
    P(i)
    w(i)=w(i-1)+alpha*g;
    w(i)
end

plot(measurements,'xk')
hold on
plot(w,'-r')
plot(P,'-g')
grid on


legend('meas','w','P','rejected','average','variance','std dev','average+-std dev')
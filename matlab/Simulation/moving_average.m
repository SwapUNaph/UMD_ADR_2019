close all
clc
clear
figure('units','normalized','outerposition',[0 0 1 1])

measurements = [-3;-3];
m1=randn(2,20)/2-3.5;
m2=randn(2,20)/4-4.2;
m3=randn(2,20)/6-3.7;
m4=randn(2,20)/8-4;
r1=randn(2,5)*2-3.5;
r2=randn(2,5)*2-3.5;
r3=randn(2,5)*2-3.5;
r4=randn(2,5)*2-3.5;
r5=randn(2,5)*2-3.5;
measurements=[measurements,r1,m1,r2,m2,r3,m3,r4,m4,r5];

EMA = zeros(size(measurements));
EMA(:,1) = measurements(:,1);

EMVar = zeros(size(measurements));
EMVar(:,1) = [2;2];

for i=2:size(measurements,2)
    delta = measurements(:,i)-EMA(:,i-1);
    std = abs(delta) ./ sqrt(EMVar(:,i-1));
    
    alpha = 0.4.^std;
    alpha = min(alpha, 0.4);
%     alpha = max(alpha, 0.001);
    EMA(:,i) = EMA(:,i-1)+alpha.*delta;
    EMVar(:,i) = (1-alpha).*(EMVar(:,i-1)+alpha.*delta.*delta);

end

EWSD = sqrt(EMVar);

plot(measurements(1,:),'xk')
hold on
plot(EMA(1,:),'-r')
plot(EMVar(1,:),'-g')
plot(EWSD(1,:),'-m')
grid on
plot(EMA(1,:)-EWSD(1,:),'b--')
plot(EMA(1,:)+EWSD(1,:),'b--')
plot(measurements(1,:),'xb')
axis([0,length(measurements),-10,5])

legend('measurements','average','variance','std dev','average+-std dev')
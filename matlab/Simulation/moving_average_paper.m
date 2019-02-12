close all
clc
clear
figure('units','normalized','outerposition',[0 0 1 1])
set(0,'defaulttextinterpreter','latex')

measurements = -4;
m1=randn(1,4)/8-4;
m2=randn(1,4)/8-4;
m3=randn(1,2)/8-4;
m4=randn(1,100)/8-4;
r1=1.9-4;
r2=randn(1,1)*0-3.5;
r3=randn(1,1)*0-3.5;
r4=randn(1,1)*0-3.5;
r5=randn(1,1)*0-3.5;
measurements=[measurements,r1,m1,r2,m2,r3,m3,m4]+4;

EMA = zeros(size(measurements));
EMA(:,1) = measurements(:,1);

EMVar = zeros(size(measurements));
EMVar(:,1) = 1;

for i=2:size(measurements,2)
    lambda = 0.2;
    
    delta = measurements(:,i)-EMA(:,i-1);
    std = abs(delta) ./ sqrt(EMVar(:,i-1));
    
    alpha = lambda.^std;
    alpha = min(alpha, lambda);
%     alpha = max(alpha, 0.001);
    EMA(:,i) = EMA(:,i-1)+alpha.*delta;
    EMVar(:,i) = (1-alpha).*(EMVar(:,i-1)+alpha.*delta.*delta);

end

EWSD = sqrt(EMVar);

plot(0,measurements(1,1),'xk')
hold on
plot(1:i-1,measurements(1,2:end),'xb')
plot(0:i-1,EMA(1,:),'-r')
% plot(EMVar(1,:),'-g')
% plot(EWSD(1,:),'-m')
grid on
plot(0:i-1,EMA(1,:)-3*EWSD(1,:),'b--')
plot(0:i-1,EMA(1,:)+3*EWSD(1,:),'b--')
axis([0,length(measurements)-1,-2,3])

legend('guess','measurements','average','average\pm\sigma')
xlabel('time step $i$')
close all
clc
clear
endpoint = 200;
figure('units','normalized','outerposition',[0 0 1 1])
meas_mean = [-4.0, -3];
mean_var = [0.5, 0.0; 0.0, 0.5]; % R
meas_total = [];
init_guess = [-1.0, 3.0];
x_update = init_guess';
P_init = [4.0, 0.0;0.0, 4.0];
P_update = P_init;
estimate = [init_guess'];
P = zeros(2,2,endpoint+1);
P(:,:,1) = P_init;

for i=1:endpoint
    meas = meas_mean' + 0.01*[i;i] + mean_var*randn(2,1);
    meas_total = [meas_total, meas];
    
    K_gain = P_update*inv(P_update + [0.1, 0.0; 0.0, 0.1]);
    
    x_update = x_update + K_gain*(meas - x_update);
    P_update = (eye(2) - K_gain)*P_update;
    
    estimate = [estimate, x_update];
    P(:,:,i+1) = P_update;
end
std = squeeze(sqrt(P(1,1,:)))';
plot(repmat(meas_mean(1),1,endpoint+1)+[1:endpoint+1]'*0.01,'-k')
hold on
plot(meas_total(1,:),'xk')
hold on
plot(estimate(1,:),'xb')
hold on

% plot(EMA(1,:),'-r')
plot(repmat(meas_mean(1),1,endpoint+1) + 3*std,'-g')
%plot(EWSD(1,:),'-m')
grid on
plot(repmat(meas_mean(1),1,endpoint+1) - 3*std,'-g')
axis([0,length(estimate),-10,5])
legend('truth', 'measurements','estimates','3 std')
hold off
std = squeeze(sqrt(P(2,2,:)))';
figure
plot(repmat(meas_mean(2),1,endpoint+1)+[1:endpoint+1]'*0.01,'-k')
hold on
plot(meas_total(2,:),'xk')
hold on
plot(estimate(2,:),'xb')
hold on
%plot(EMA(1,:),'-r')
plot(repmat(meas_mean(2),1,endpoint+1) + 3*std,'-g')
%plot(EWSD(1,:),'-m')
grid on
plot(repmat(meas_mean(2),1,endpoint+1) - 3*std,'-g')
axis([0,length(estimate),-10,5])
legend('truth', 'measurements','estimates','3 std')
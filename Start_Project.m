close all
clear 
clc
duration = input('How long do you want to simluate: ');

[X, meas] = filterTemplateLog(0, 0, 0, duration);

Euler_measurement = quat2eul(meas.orient');
Euler_estimate = quat2eul(X.x');

figure
subplot(3,1,1)
hold on
grid on
plot(meas.t, Euler_measurement(:,1))
plot(meas.t, Euler_estimate(:,1))
legend('True','Estimate')
xlabel('rad')
ylabel('t')
title('Roll')
hold off

subplot(3,1,2)
hold on
grid on
plot(meas.t, Euler_measurement(:,2))
plot(meas.t, -1 * Euler_estimate(:,3))
legend('True','Estimate')
title('Pitch')
xlabel('rad')
ylabel('t')
hold off

subplot(3,1,3)
hold on
grid on
plot(meas.t, Euler_measurement(:,3))
plot(meas.t, Euler_estimate(:,2))
legend('True','Estimate')
title('Yaw')
xlabel('rad')
ylabel('t')
hold off

%% SSY345 Group 15
clc
clear all
close all
data = load('sensorlog_flat.mat');

% Acceleration data
acc_x = table2array(data.Acceleration(30:535,1));
acc_y = table2array(data.Acceleration(30:535,2));
acc_z = table2array(data.Acceleration(30:535,3));

acc_x_mean = mean(acc_x);
acc_y_mean = mean(acc_y);
acc_z_mean = mean(acc_z);

acc_x_var = var(acc_x);
acc_y_var = var(acc_y);
acc_z_var = var(acc_z);

figure
hold on
plot(acc_x);
plot(acc_y);
plot(acc_z);
title('Acceleration data')
legend('x', 'y', 'z', 'location', 'north')
xlabel('0.01s')
ylabel('m/s^2')
hold off

figure
hold on
histogram(acc_x,50)
title('Acceleration data distribution in the x-axis')
xlabel('m/s^2')
ylabel('Number of data points')
hold off

figure
hold on
histogram(acc_y,50)
title('Acceleration data distribution in the y-axis')
xlabel('m/s^2')
ylabel('Number of data points')
hold off

figure
hold on
histogram(acc_z,50)
title('Acceleration data distribution in the z-axis')
xlabel('m/s^2')
ylabel('Number of data points')
hold off


% Magnetic data
mag_x = table2array(data.MagneticField(30:535,1));
mag_y = table2array(data.MagneticField(30:535,2));
mag_z = table2array(data.MagneticField(30:535,3));

mag_x_mean = mean(mag_x);
mag_y_mean = mean(mag_y);
mag_z_mean = mean(mag_z);

mag_x_var = var(mag_x);
mag_y_var = var(mag_y);
mag_z_var = var(mag_z);

figure
hold on
plot(mag_x);
plot(mag_y);
plot(mag_z);
title('Magnetic data')
legend('x', 'y', 'z', 'location', 'north')
xlabel('0.01s')
ylabel('mu*T')
hold off

figure
hold on
histogram(mag_x,50)
title('Magnetic data distribution in the x-axis')
xlabel('mu*T')
ylabel('Number of data points')
hold off

figure
hold on
histogram(mag_y,50)
title('Magnetic data distribution in the y-axis')
xlabel('mu*T')
ylabel('Number of data points')
hold off

figure
hold on
histogram(mag_z,50)
title('Magnetic data distribution in the z-axis')
xlabel('mu*T')
ylabel('Number of data points')
hold off


% Gyro data
gyro_x = table2array(data.AngularVelocity(30:535,1));
gyro_y = table2array(data.AngularVelocity(30:535,2));
gyro_z = table2array(data.AngularVelocity(30:535,3));

gyro_x_mean = mean(gyro_x);
gyro_y_mean = mean(gyro_y);
gyro_z_mean = mean(gyro_z);

gyro_x_var = var(gyro_x);
gyro_y_var = var(gyro_y);
gyro_z_var = var(gyro_z);

figure
hold on
plot(gyro_x);
plot(gyro_y);
plot(gyro_z);
title('Gyro data')
legend('x', 'y', 'z', 'location', 'north')
xlabel('0.01s')
ylabel('omega/s')
hold off

figure
hold on
histogram(gyro_x,50)
title('Gyro data distribution in the x-axis')
xlabel('Angular velocity')
ylabel('Number of data points')
hold off

figure
hold on
histogram(gyro_y,50)
title('Gyro data distribution in the y-axis')
xlabel('Angular velocity')
ylabel('Number of data points')
hold off

figure
hold on
histogram(gyro_z,50)
title('Gyro data distribution in the z-axis')
xlabel('Angular velocity')
ylabel('Number of data points')
hold off
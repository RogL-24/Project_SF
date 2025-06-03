%% SSY345 Group 15
clc
clear
close all

data = load('sensorlog_flat.mat');
nrbin = 50; % number of bins

T = 1/100; %1/f 100 Hz
N = min([length(table2array(data.Acceleration)), length(table2array(data.AngularVelocity)), length(table2array(data.MagneticField)), length(table2array(data.Orientation))]); % no of points
total_time = 0:T:T*N - T ; % Time interval
g = 9.786007570512820; % z mean from Dataset3.mat

%% Acceleration data - Task 2
acc_x = table2array(data.Acceleration(1:N,1));
acc_y = table2array(data.Acceleration(1:N,2));
acc_z = table2array(data.Acceleration(1:N,3));
acc = [acc_x, acc_y, acc_z];

acc_x_mean = mean(acc_x);
acc_y_mean = mean(acc_y);
acc_z_mean = mean(acc_z);

acc_x_var = var(acc_x);
acc_y_var = var(acc_y);
acc_z_var = var(acc_z);
acc_cov = cov(acc);

figure
hold on
grid on
plot(total_time, acc_x);
plot(total_time, acc_y);
plot(total_time, acc_z);
title('Acceleration data')
legend('x', 'y', 'z', 'location', 'northeast')
xlabel('0.01s')
ylabel('m/s^2')
hold off

figure
subplot(3,1,1)
hold on
grid on
histogram(acc_x, nrbin)
title('Acceleration data distribution in the x-axis')
xlabel('m/s^2')
ylabel('No of data points')
hold off

subplot(3,1,2)
hold on
grid on
histogram(acc_y, nrbin)
title('Acceleration data distribution in the y-axis')
xlabel('m/s^2')
ylabel('No of data points')
hold off

subplot(3,1,3)
hold on
grid on
histogram(acc_z, nrbin)
title('Acceleration data distribution in the z-axis')
xlabel('m/s^2')
ylabel('No of data points')
hold off


%% Magnetic data - Task 2
mag_x = table2array(data.MagneticField(1:N,1));
mag_y = table2array(data.MagneticField(1:N,2));
mag_z = table2array(data.MagneticField(1:N,3));
mag = [mag_x, mag_y, mag_z];

mag_x_mean = mean(mag_x);
mag_y_mean = mean(mag_y);
mag_z_mean = mean(mag_z);

mag_x_var = var(mag_x);
mag_y_var = var(mag_y);
mag_z_var = var(mag_z);
mag_cov = cov(mag);

figure
hold on
grid on
plot(total_time, mag_x);
plot(total_time, mag_y);
plot(total_time, mag_z);
title('Magnetic data')
legend('x', 'y', 'z', 'location', 'northeast')
xlabel('0.01s')
ylabel('\muT')
hold off

figure
subplot(3,1,1)
grid on
hold on
histogram(mag_x, nrbin)
title('Magnetic data distribution in the x-axis')
xlabel('\muT')
ylabel('No. of points')
hold off


subplot(3,1,2)
grid on
hold on
histogram(mag_y, nrbin)
title('Magnetic data distribution in the y-axis')
xlabel('\muT')
ylabel('No. of points')
hold off

subplot(3,1,3)
grid on
hold on
histogram(mag_z, nrbin)
title('Magnetic data distribution in the z-axis')
xlabel('\muT')
ylabel('No. of points')
hold off
 
 
%% Gyro data - Task 2
gyro_x = table2array(data.AngularVelocity(1:N,1));
gyro_y = table2array(data.AngularVelocity(1:N,2));
gyro_z = table2array(data.AngularVelocity(1:N,3));
gyro = [gyro_x, gyro_y, gyro_z];

gyro_x_mean = mean(gyro_x);
gyro_y_mean = mean(gyro_y);
gyro_z_mean = mean(gyro_z);
gyro_mean = [gyro_x_mean, gyro_y_mean, gyro_z_mean];

gyro_x_var = var(gyro_x);
gyro_y_var = var(gyro_y);
gyro_z_var = var(gyro_z);
gyro_cov = cov(gyro);

figure
hold on
grid on
plot(total_time, gyro_x);
plot(total_time, gyro_y);
plot(total_time, gyro_z);
title('Gyro data')
legend('x', 'y', 'z', 'location', 'northeast')
xlabel('0.01s')
ylabel('\omega/s')
hold off

figure
subplot(3,1,1)
grid on
hold on
histogram(gyro_x, nrbin)
title('Gyro data distribution in the x-axis')
xlabel('Angular velocity')
ylabel('Number of data points')
hold off

subplot(3,1,2)
grid on
hold on
histogram(gyro_y, nrbin)
title('Gyro data distribution in the y-axis')
xlabel('Angular velocity')
ylabel('Number of data points')
hold off

subplot(3,1,3)
grid on
hold on
histogram(gyro_z, nrbin)
title('Gyro data distribution in the z-axis')
xlabel('Angular velocity')
ylabel('Number of data points')
hold off
 
%% task 5
n = length(gyro_x);
x_Pred = zeros(4,n);
P_Pred = zeros(4, 4, n);
q0 = Somega(gyro_mean) * [1;1;1;1];
P0 = eye(4);
x_Pred(:,1) = q0;
P_Pred(:,:,1) = P0;

for i = 2:n
    [x_Pred(:,i), P_Pred(:,:,i)] = tu_qw(x_Pred(:,i-1), P_Pred(:,:,i-1), gyro(i-1,:)', T, gyro_cov);
    [x_Pred(:,i), P_Pred(:,:,i)] = mu_normalizeQ(x_Pred(:,i), P_Pred(:,:,i));
end

figure
hold on
plot(total_time, x_Pred(1,:));
plot(total_time, x_Pred(2,:));
plot(total_time, x_Pred(3,:));
plot(total_time, x_Pred(4,:));
title('EKF w/o measurement update')
legend('q1', 'q2', 'q3', 'q4')
xlabel('0.01s')
ylabel('q')
hold off

% 
%% Task 6 & 7
g0 = [acc_x_mean; acc_y_mean; acc_z_mean];
x_Post_wa = zeros(4,n); % x Posterior with accelerometer data
P_Post_wa = zeros(4, 4, n); % P Posterior with accelerometer data
% q0 = Somega(gyro_mean) * [1;1;1;1];
q0 = [1; 0; 0; 0];
P0 = eye(4);
x_Post_wa(:,1) = q0;
P_Post_wa(:,:,1) = P0;


for i = 2:n
    [x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1)] = tu_qw(x_Post_wa(:,i-1), P_Post_wa(:,:,i-1), gyro(i-1,:)', T, gyro_cov);
    [x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1)] = mu_normalizeQ(x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1));
    
    [x_Post_wa(:,i), P_Post_wa(:,:,i)] = mu_g(x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1), acc(i-1,:)', acc_cov, g0);
    [x_Post_wa(:,i), P_Post_wa(:,:,i)] = mu_normalizeQ(x_Post_wa(:,i), P_Post_wa(:,:,i));
    
    estimated_orientation_wa(:,i) = q2euler(x_Post_wa(:,i));
end

figure
hold on
plot(total_time, x_Post_wa(1,:));
plot(total_time, x_Post_wa(2,:));
plot(total_time, x_Post_wa(3,:));
plot(total_time, x_Post_wa(4,:));
title('EKF updated with accelerometer')
legend('q1', 'q2', 'q3', 'q4')
xlabel('0.01s')
ylabel('q')
hold off

orient_x = table2array(data.Orientation(:,1));
orient_y = table2array(data.Orientation(:,2));
orient_z = table2array(data.Orientation(:,3));

figure
subplot(3,1,1)
grid on
hold on
plot(total_time, orient_x(1:N)');
plot(total_time, estimated_orientation_wa(1,:));
title('Estimated vs True orientation : x (gyro & acc)')
legend('True orientation', 'Estimated orientation')
hold off

subplot(3,1,2) 
grid on
hold on
plot(total_time, orient_y(1:N)');
plot(total_time, estimated_orientation_wa(2,:));
title('Estimated vs True orientation : y (gyro & acc)')
legend('True orientation', 'Estimated orientation')
hold off

subplot(3,1,3) 
grid on
hold on
plot(total_time, orient_z(1:N)');
plot(total_time, estimated_orientation_wa(3,:));
title('Estimated vs True orientation : z (gyro & acc)')
legend('True orientation', 'Estimated orientation')
hold off


%% Task 8

beta = 0.05; % the limit for assuming that an acceleration is happening in 
             % Outliner 1
             % 0.05 best for outliner 1

x_Post_wa(:,1) = q0;
P_Post_wa(:,:,1) = P0;

acc_edit = zeros(N,3);
L_hat_acc = zeros(1,N);
acc_limit = 0.5;
alpha = 0.5;

for i = 2:n
    [x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1)] = tu_qw(x_Post_wa(:,i-1), P_Post_wa(:,:,i-1), gyro(i-1,:)', T, gyro_cov);
    [x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1)] = mu_normalizeQ(x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1));
    
    % % Outliner vesrion 1
    % if i >= 3
    %     gyro_x_diff = gyro(i-1,1)-gyro(i-2,1);
    %     gyro_y_diff = gyro(i-1,2)-gyro(i-2,2);
    %     gyro_z_diff = gyro(i-1,3)-gyro(i-2,3);
    % else
    %     gyro_x_diff = gyro(i-1,1)-0;
    %     gyro_y_diff = gyro(i-1,2)-0;
    %     gyro_z_diff = gyro(i-1,3)-0;
    % end
    % 
    % abs_gyro = sqrt(gyro_x_diff^2 + gyro_y_diff^2+ gyro_z_diff^2);
    % 
    % if  abs_gyro < beta
    %     acc_edit(i-1,3) = g;
    % else
    %     acc_edit(i-1,:) = acc(i-1,:);
    % end
    % [x_Post_wa(:,i), P_Post_wa(:,:,i)] = mu_g(x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1), acc_edit(i-1,:)', acc_cov, g0);
    % [x_Post_wa(:,i), P_Post_wa(:,:,i)] = mu_normalizeQ(x_Post_wa(:,i), P_Post_wa(:,:,i));

    % outliner v2

    acc_norm = sqrt(acc(i-1,1)^2 + acc(i-1,2)^2 + acc(i-1,3)^2);
    L_hat_acc(i) = (1 - alpha)*L_hat_acc(i-1) + alpha * acc_norm;
    
    if abs(acc_norm - L_hat_acc) < acc_limit
        [x_Post_wa(:,i), P_Post_wa(:,:,i)] = mu_g(x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1), acc(i-1,:)', acc_cov, g0);
        [x_Post_wa(:,i), P_Post_wa(:,:,i)] = mu_normalizeQ(x_Post_wa(:,i), P_Post_wa(:,:,i));

    else % acc data is skipped
        x_Post_wa(:,i) = x_Pred_wa(:,i-1);
        P_Post_wa(:,:,i) = P_Pred_wa(:,:,i-1);
    end

    
    
    estimated_orientation_wa(:,i) = q2euler(x_Post_wa(:,i)); 
end


figure
hold on
plot(total_time, x_Post_wa(1,:));
plot(total_time, x_Post_wa(2,:));
plot(total_time, x_Post_wa(3,:));
plot(total_time, x_Post_wa(4,:));
title('EKF updated with edited accelerometer')
legend('q1', 'q2', 'q3', 'q4')
xlabel('0.01s')
ylabel('q')
hold off

orient_x = table2array(data.Orientation(:,1));
orient_y = table2array(data.Orientation(:,2));
orient_z = table2array(data.Orientation(:,3));



figure
subplot(3,1,1)
grid on
hold on
plot(total_time, orient_x(1:N)');
plot(total_time, estimated_orientation_wa(1,:));
title('Estimated vs True orientation : x (gyro & edit acc)')
legend('True orientation', 'Estimated orientation')
hold off

subplot(3,1,2) 
grid on
hold on
plot(total_time, orient_y(1:N)');
plot(total_time, estimated_orientation_wa(2,:));
title('Estimated vs True orientation : y (gyro & edit acc)')
legend('True orientation', 'Estimated orientation')
hold off

subplot(3,1,3) 
grid on
hold on
plot(total_time, orient_z(1:N)');
plot(total_time, estimated_orientation_wa(3,:));
title('Estimated vs True orientation : z (gyro & edit acc)')
legend('True orientation', 'Estimated orientation')
hold off




%% task 9 & 10
x_Post_wm = zeros(4,n); % x Posterior with magnetometer data
P_Post_wm = zeros(4, 4, n); % P Posterior with magnetometer data
x_Post_wm(:,1) = q0;
P_Post_wm(:,:,1) = P0;
m0 = zeros(3,N);

for i = 2:n
    [x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1)] = tu_qw(x_Post_wm(:,i-1), P_Post_wm(:,:,i-1), gyro(i-1,:)', T, gyro_cov);
    [x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1)] = mu_normalizeQ(x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1));
    
    m0(:,i-1) = [0; sqrt(mag_x(i-1,1)^2 + mag_y(i-1,1)^2); mag_z(i-1,1)];

    [x_Post_wm(:,i), P_Post_wm(:,:,i)] = mu_m(x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1), mag(i-1,:)', m0(:,i-1), mag_cov);
    [x_Post_wm(:,i), P_Post_wm(:,:,i)] = mu_normalizeQ(x_Post_wm(:,i), P_Post_wm(:,:,i));

    estimated_orientation_wm(:,i) = q2euler(x_Post_wm(:,i));
end

figure
hold on
plot(total_time, x_Post_wm(1,:));
plot(total_time, x_Post_wm(2,:));
plot(total_time, x_Post_wm(3,:));
plot(total_time, x_Post_wm(4,:));
title('EKF updated with mag')
legend('q1', 'q2', 'q3', 'q4')
xlabel('0.01s')
ylabel('q')
hold off

orient_x = table2array(data.Orientation(:,1));
orient_y = table2array(data.Orientation(:,2));
orient_z = table2array(data.Orientation(:,3));

figure
subplot(3,1,1)
grid on
hold on
plot(total_time, orient_x(1:N));
plot(total_time, estimated_orientation_wm(1,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : x (gyro & mag)')
hold off

subplot(3,1,2) 
grid on
hold on
plot(total_time, orient_y(1:N));
plot(total_time, estimated_orientation_wm(2,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : y (gyro & mag)')
hold off

subplot(3,1,3)
grid on
hold on
plot(total_time, orient_z(1:N));
plot(total_time, estimated_orientation_wm(3,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : z (gyro & mag)')
hold off



%% Task 11
alpha = 0.01; %tune for estimatded magnitud
mag_limit = 0.5; % limit for mag
x_Post_wm = zeros(4,n); % x Posterior with magnetometer data
P_Post_wm = zeros(4, 4, n); % P Posterior with magnetometer data
x_Post_wm(:,1) = q0;
P_Post_wm(:,:,1) = P0;
m0 = zeros(3,N);
m0_norm = zeros(1,N-1);
m_norm = zeros(1,N-1);
L_hat = zeros(1,N-1);
L_hat(1) = sqrt(mag(1,1)^2 + mag(1,2)^2 + mag(1,3)^2);

for i = 2:n
    [x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1)] = tu_qw(x_Post_wm(:,i-1), P_Post_wm(:,:,i-1), gyro(i-1,:)', T, gyro_cov);
    [x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1)] = mu_normalizeQ(x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1));
    
    m0(:,i-1) = [0; sqrt(mag_x(i-1,1)^2 + mag_y(i-1,1)^2); mag_z(i-1,1)];
    m0_norm(i-1) = sqrt(m0(2,i-1)^2 + m0(3,i-1)^2);  % m0 norm and m norm is the same
    m_norm(i-1) = sqrt(mag(i-1,1)^2 + mag(i-1,2)^2 + mag(i-1,3)^2);

    L_hat(i) = (1 - alpha) * L_hat(i-1) + alpha * m_norm(i-1);

    if mag_limit > abs(m_norm(i-1) - L_hat(i))
        [x_Post_wm(:,i), P_Post_wm(:,:,i)] = mu_m(x_Pred_wm(:,i-1), P_Pred_wm(:,:,i-1), mag(i-1,:)', m0(:,i-1), mag_cov);
        [x_Post_wm(:,i), P_Post_wm(:,:,i)] = mu_normalizeQ(x_Post_wm(:,i), P_Post_wm(:,:,i));
    else % skip the mag update
        x_Post_wm(:,i) = x_Pred_wm(:,i-1);
        P_Post_wm(:,:,i) = P_Pred_wm(:,:,i-1);
    end

    estimated_orientation_wm(:,i) = q2euler(x_Post_wm(:,i));
end

m0_norm_mean = mean(m0_norm);
m_norm_mean = mean(m_norm);

figure
hold on
plot(m0_norm);
plot(m_norm);
title('norm')
legend('m0', 'm')
hold off

figure
hold on
plot(total_time, x_Post_wm(1,:));
plot(total_time, x_Post_wm(2,:));
plot(total_time, x_Post_wm(3,:));
plot(total_time, x_Post_wm(4,:));
title('EKF updated with mag')
legend('q1', 'q2', 'q3', 'q4')
xlabel('0.01s')
ylabel('q')
hold off

orient_x = table2array(data.Orientation(:,1));
orient_y = table2array(data.Orientation(:,2));
orient_z = table2array(data.Orientation(:,3));

figure
subplot(3,1,1)
grid on
hold on
plot(total_time, orient_x(1:N));
plot(total_time, estimated_orientation_wm(1,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : x (gyro & mag)')
hold off

subplot(3,1,2) 
grid on
hold on
plot(total_time, orient_y(1:N));
plot(total_time, estimated_orientation_wm(2,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : y (gyro & mag)')
hold off

subplot(3,1,3)
grid on
hold on
plot(total_time, orient_z(1:N));
plot(total_time, estimated_orientation_wm(3,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : z (gyro & mag)')
hold off

%% Update using both accelerometer and magnetometer
g0 = [acc_x_mean; acc_y_mean; acc_z_mean];
x_Post = zeros(4,n); % x Posterior with accelerometer data
P_Post = zeros(4, 4, n); % P Posterior with accelerometer data

q0 = [1; 0; 0; 0];
P0 = eye(4);
x_Post(:,1) = q0;
P_Post(:,:,1) = P0;


for i = 2:n
    [x_Pred(:,i-1), P_Pred(:,:,i-1)] = tu_qw(x_Post(:,i-1), P_Post(:,:,i-1), gyro(i-1,:)', T, gyro_cov);
    [x_Pred(:,i-1), P_Pred(:,:,i-1)] = mu_normalizeQ(x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1));
    
    % Accelerometer update
    [x_Post(:,i), P_Post(:,:,i)] = mu_g(x_Pred_wa(:,i-1), P_Pred_wa(:,:,i-1), acc(i,:)', acc_cov, g0);
    [x_Post(:,i), P_Post(:,:,i)] = mu_normalizeQ(x_Post(:,i), P_Post(:,:,i));

    % Magnetometer update
    [x_Post(:,i), P_Post(:,:,i)] = mu_m(x_Post(:,i), P_Post(:,:,i), mag(i,:)', m0(:,i-1), mag_cov);
    [x_Post(:,i), P_Post(:,:,i)] = mu_normalizeQ(x_Post(:,i), P_Post(:,:,i));

    estimated_orientation_wa(:,i) = q2euler(x_Post(:,i));
end

figure
hold on
plot(total_time, x_Post_wm(1,:));
plot(total_time, x_Post_wm(2,:));
plot(total_time, x_Post_wm(3,:));
plot(total_time, x_Post_wm(4,:));
title('EKF updated with acc & mag')
legend('q1', 'q2', 'q3', 'q4')
xlabel('0.01s')
ylabel('q')
hold off

orient_x = table2array(data.Orientation(:,1));
orient_y = table2array(data.Orientation(:,2));
orient_z = table2array(data.Orientation(:,3));

figure
subplot(3,1,1)
grid on
hold on
plot(total_time, orient_x(1:N));
plot(total_time, estimated_orientation_wm(1,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : x (gyro, acc & mag)')
hold off

subplot(3,1,2) 
grid on
hold on
plot(total_time, orient_y(1:N));
plot(total_time, estimated_orientation_wm(2,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : y (gyro, acc & mag)')
hold off

subplot(3,1,3)
grid on
hold on
plot(total_time, orient_z(1:N));
plot(total_time, estimated_orientation_wm(3,:));
legend('True orientation', 'Estimated orientation')
title('Estimated vs True orientation : z (gyro, acc & mag)')
hold off
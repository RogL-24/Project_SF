clear; close all; clc;
%% ---------- TUNING PARAMETERS ---------- %%
Ra = [ 0.0003, 0.0005, -0.0000;   
       0.0005, 0.0012, -0.0000;
      -0.0000, -0.0000, 0.0001 ];

Rw = 1e-5 * [ 0.2076, -0.0219, 0.0065;
             -0.0219, 0.2099, -0.0312;
              0.0065, -0.0312, 0.2954 ];

Rm = [ 0.0378, 0.0000, 0.0025;
       0.0000, 0.0380, -0.0062;
       0.0025, -0.0062, 0.1314 ] * 10;

g0 = [-0.0334; 0.0521; 9.7763];
m0 = [0; 15.6164; -47.0349];

fprintf('=== USING TUNING PARAMETERS ===\n');
fprintf('g0 = [%.4f; %.4f; %.4f]\n', g0);
fprintf('m0 = [%.4f; %.4f; %.4f]\n\n', m0);

%% ---------- Initialize Mobile Device ---------- %%
clear m;
m = mobiledev;
m.AccelerationSensorEnabled = 1;
m.AngularVelocitySensorEnabled = 1;
m.MagneticSensorEnabled = 1;

m.OrientationSensorEnabled = 1; % Use thisreference to evaluate filter  

mode = input('Enter log duration in seconds (or type 0 to stream until manual stop): ');
T = 0.01;
nx = 4;
x = [1; 0; 0; 0];
P = eye(nx);

logDuration = mode;
if logDuration == 0
    xlim_duration = 9999;
else
    xlim_duration = logDuration;
end

%% ---------- Setup for live-plotting ---------- %%

% Figure for euler angels
figure(1); clf;
hYaw   = animatedline('Color','r','LineWidth',1.5);
hPitch = animatedline('Color','b','LineWidth',1.5);
hRoll  = animatedline('Color','g','LineWidth',1.5);
xlabel('Time (s)');
ylabel('Degrees');
legend('Yaw','Pitch','Roll');
title('EKF Orientation Estimate');
grid on;
xlim([0, xlim_duration]);
ylim([-180, 180]);
drawnow;

% Figure for 3D visualization
figure(2); clf;
axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-1 1]); ylim([-1 1]); zlim([-1 1]);
view(3);

phoneVerts = 0.1 * [
    -1 -0.5 -0.1;
     1 -0.5 -0.1;
     1  0.5 -0.1;
    -1  0.5 -0.1;
    -1 -0.5  0.1;
     1 -0.5  0.1;
     1  0.5  0.1;
    -1  0.5  0.1
];
phoneFaces = [
    1 2 3 4; 5 6 7 8;
    1 2 6 5; 2 3 7 6;
    3 4 8 7; 4 1 5 8
];

hBox = patch('Vertices', phoneVerts, 'Faces', phoneFaces, ...
    'FaceColor', [0.8 0.8 1], 'FaceAlpha', 0.7);

axisLength = 0.3;
hY = line([0 axisLength], [0 0], [0 0], 'Color', 'r', 'LineWidth', 2);  % Y = red
hX = line([0 0], [0 axisLength], [0 0], 'Color', 'g', 'LineWidth', 2);  % X = green
hZ = line([0 0], [0 0], [0 axisLength], 'Color', 'b', 'LineWidth', 2);  % Z = blue


x0 = 0; y0 = 0;  z0 = 0; % Phone starting position


%% ----------Initialization ---------- %%
maxSamples = 100000;
q = zeros(4, maxSamples);
time_vec = zeros(1, maxSamples);
eul_deg = zeros(3, maxSamples);
k = 0;

g = 9.81;
acc_threshold = 2;

alpha = 0.01;
mag_threshold = 5;      % Adjust based on expected deviation (in µT)

q_builtin = zeros(4, maxSamples);      % From phone orientation
quat_error_deg = zeros(1, maxSamples); 

%% ---------- Start Logging ---------- %%
disp('Starting live EKF... Move your phone now.');
m.Logging = 1;
tStart = tic;

while (logDuration == 0 && m.Logging) || (logDuration > 0 && toc(tStart) < logDuration)
    acc = m.Acceleration;
    gyr = m.AngularVelocity;
    mag = m.MagneticField;
    ori = m.Orientation;

    if isempty(acc) || isempty(gyr) || isempty(mag) || any(isnan(acc)) || any(isnan(mag)) || isempty(ori) || any(isnan(ori)) 
        pause(T);
        continue;
    end

    acc = acc(:); gyr = gyr(:); mag = mag(:);
    k = k + 1;
    time_vec(k) = toc(tStart);

    if all(~isnan(gyr))
        [x, P] = tu_qw(x, P, gyr, T, Rw);
    else
        F = eye(4);
        G = 0.5 * T * Sq(x);
        x = F * x;
        P = F * P * F' + G * Rw * G';
        [x, P] = mu_normalizeQ(x, P);
    end

    if all(~isnan(acc))
        acc_norm = norm(acc);
        % Outlier rejection
        if abs(acc_norm - g) < acc_threshold
            [x, P] = mu_g(x, P, acc, Ra, g0); % Define Ra and g0
        else
            disp('Skipping acc update..')
        end
    end
   if abs(norm(acc) - norm(g0)) < 2.0
        [x, P] = mu_g(x, P, acc, Ra, g0);
    end

    if all(~isnan(mag))
        if k == 1
            L_hat = norm(mag);  % Initialize once
        else
            L_hat = (1 - alpha) * L_hat + alpha * norm(mag);
        end
        mag_threshold = 0.5;      % Adjust based on expected deviation (in µT)

        % Update expected field strength
        L_hat = (1 - alpha) * L_hat + alpha * norm(mag);

        % Outlier check
        if abs(norm(mag) - L_hat) < mag_threshold
            magOut = 0;
            [x, P] = mu_m(x, P, mag, m0, Rm); % Define Rm and m0
        else
            disp('Skipping mag update..')
        end
    end

    q(:, k) = x;

    if all(~isnan(ori))
        % Convert [azimuth, pitch, roll] to quaternion
        % MATLAB's eul2quat expects [yaw pitch roll] in radians
        eul_rad_builtin = deg2rad([ori(1), ori(2), ori(3)]);  % [yaw pitch roll]
        q_ref = eul2quat(eul_rad_builtin, 'ZYX');  % [w x y z]
    
        q_builtin(:, k) = q_ref(:);
    
        % Compute angular error between EKF and phone quaternion
        q1 = q(:,k);       % EKF
        q2 = q_ref(:);     % Reference
        angle_error_rad = 2 * acos(abs(dot(q1, q2)));
        quat_error_deg(k) = rad2deg(angle_error_rad);
    end

    R = quat2rotm_custom(x);
    R = R * [0 1 0; 1 0 0; 0 0 1]; 
  
    rotatedVerts = (R * phoneVerts')';
    %phonePosition = [x0, y0, z0];  % Replace with desired start location
    %rotatedVerts = (R * phoneVerts')' + phonePosition;

    set(hBox, 'Vertices', rotatedVerts);
    set(hX, 'XData', [0 R(1,1)*axisLength], 'YData', [0 R(2,1)*axisLength], 'ZData', [0 R(3,1)*axisLength]);
    set(hY, 'XData', [0 R(1,2)*axisLength], 'YData', [0 R(2,2)*axisLength], 'ZData', [0 R(3,2)*axisLength]);
    set(hZ, 'XData', [0 R(1,3)*axisLength], 'YData', [0 R(2,3)*axisLength], 'ZData', [0 R(3,3)*axisLength]);

    eul = quat2eul_custom(x'); % Returns [roll, pitch, yaw]
    eul_deg(:, k) = rad2deg(eul);

    addpoints(hRoll,  time_vec(k), eul_deg(1,k));  % Roll = index 1
    addpoints(hPitch, time_vec(k), eul_deg(2,k));  % Pitch = index 2  
    addpoints(hYaw,   time_vec(k), eul_deg(3,k));  % Yaw = index 3

    if mod(k, 5) == 0
        drawnow limitrate;
    end

    pause(T);
end

m.Logging = 0;
disp('Finished live EKF.');

q = q(:, 1:k);
time_vec = time_vec(1:k);
eul_deg = eul_deg(:, 1:k);
%% ---------- Plots ---------- %%
figure(3);
plot(time_vec, q');
legend('w', 'x', 'y', 'z', 'Location', 'best');
title('EKF Quaternion Components');
xlabel('Time (s)'); ylabel('Value');
grid on;

figure(4);
plot(time_vec, eul_deg(3,:), 'r', 'LineWidth', 2); hold on;
plot(time_vec, eul_deg(2,:), 'b', 'LineWidth', 2);
plot(time_vec, eul_deg(1,:), 'g', 'LineWidth', 2);
legend('Yaw', 'Pitch', 'Roll', 'Location', 'best');
title('EKF Orientation Estimate');
xlabel('Time (s)'); ylabel('Degrees');
grid on;

q_builtin = q_builtin(:, 1:k);
quat_error_deg = quat_error_deg(1:k);

% Plot quaternion error
figure(5);
plot(time_vec, quat_error_deg, 'LineWidth', 2);
title('Quaternion Angular Error (Degrees)');
xlabel('Time (s)');
ylabel('Angular Error (°)');
grid on;

% Print summary stats
fprintf('Mean Quaternion Error: %.2f°\n', mean(quat_error_deg));
fprintf('Max Quaternion Error:  %.2f°\n', max(quat_error_deg));

%% ---------- Helper Functions ---------- %%

% Se över

function eul = quat2eul_custom(q)
    % Convert quaternion q = [w x y z] to Euler angles 
    % Returns [roll, pitch, yaw] in that order
    
    % Normalize quaternion
    q = q / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    % Convert to Euler angles (ZYX convention - yaw, pitch, roll)
    % Roll (rotation about X-axis)
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    pitch = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch (rotation about Y-axis) - clamp to avoid numerical issues
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        roll = sign(sinp) * pi/2; % use 90 degrees if out of range
    else
        roll = asin(sinp);
    end
    
    % Yaw (rotation about Z-axis)
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    
    eul = [roll, pitch, yaw]; % [Roll, Pitch, Yaw]
end

% Se över

function R = quat2rotm_custom(q)
    q = q / norm(q);
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    R = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
         2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
         2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
end


function [x, P] = tu_qw(x, P, omega, T, Rw)
    G = 0.5*T * Sq(x);
    F = eye(4) + 0.5*T*Somega(omega);
    x = F * x;
    P = F * P * F' + G * Rw * G';
    [x, P] = mu_normalizeQ(x, P);
end

function [x, P] = mu_g(x, P, yacc, Ra, g0)
    h = Qq(x)' * g0;
    [Q0, Q1, Q2, Q3] = dQqdq(x);
    H = [Q0'*g0, Q1'*g0, Q2'*g0, Q3'*g0];
    S = H * P * H' + Ra;
    K = P * H' / S;
    x = x + K * (yacc - h);
    P = P - K * H * P;
    [x, P] = mu_normalizeQ(x, P);
end

function [x, P] = mu_m(x, P, ymag, m0, Rm)
    h = Qq(x)' * m0;
    [Q0, Q1, Q2, Q3] = dQqdq(x);
    H = [Q0'*m0, Q1'*m0, Q2'*m0, Q3'*m0];
    S = H * P * H' + Rm;
    K = P * H' / S;
    x = x + K * (ymag - h);
    P = P - K * H * P;
    [x, P] = mu_normalizeQ(x, P);
end

function [x, P] = mu_normalizeQ(x, P)
    x = x / norm(x);
    J = eye(4) - (x * x')/(x' * x);
    P = J * P * J';
end

function S = Somega(w)
    S = [ 0 -w(1) -w(2) -w(3);
          w(1) 0 w(3) -w(2);
          w(2) -w(3) 0 w(1);
          w(3) w(2) -w(1) 0];
end

function S = Sq(q)
    S = [-q(2) -q(3) -q(4);
          q(1) -q(4) q(3);
          q(4) q(1) -q(2);
         -q(3) q(2) q(1)];
end

function [Q0, Q1, Q2, Q3] = dQqdq(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    Q0 = 2*[2*q0 -q3 q2;
            q3 2*q0 -q1;
           -q2 q1 2*q0];
    Q1 = 2*[2*q1 q2 q3;
            q2 0 -q0;
            q3 q0 0];
    Q2 = 2*[0 q1 q0;
            q1 2*q2 q3;
           -q0 q3 0];
    Q3 = 2*[0 -q0 q1;
            q0 0 q2;
            q1 q2 2*q3];
end

function Q = Qq(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    Q = [2*(q0^2+q1^2)-1, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), 2*(q0^2+q2^2)-1, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), 2*(q0^2+q3^2)-1];
end





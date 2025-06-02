function [xhat, meas] = filterTemplateLog(calAcc, calGyr, calMag)
    % FILTERTEMPLATE  Filter template
    %
    % This is a template function for how to collect and filter data
    % sent from a smartphone live.  Calibration data for the
    % accelerometer, gyroscope and magnetometer assumed available as
    % structs with fields m (mean) and R (variance).
    %
    % The function returns xhat as an array of structs comprising t
    % (timestamp), x (state), and P (state covariance) for each
    % timestamp, and meas an array of structs comprising t (timestamp),
    % acc (accelerometer measurements), gyr (gyroscope measurements),
    % mag (magnetometer measurements), and orint (orientation quaternions
    % from the phone).  Measurements not availabe are marked with NaNs.
    %
    % As you implement your own orientation estimate, it will be
    % visualized in a simple illustration.  If the orientation estimate
    % is checked in the Sensor Fusion app, it will be displayed in a
    % separate view.
    %
    % Note that it is not necessary to provide inputs (calAcc, calGyr, calMag).
    
    %% Setup necessary infrastructure
    m = mobiledev;
    m.AccelerationSensorEnabled = 1;
    m.AngularVelocitySensorEnabled = 1;
    m.MagneticSensorEnabled = 1;
    m.OrientationSensorEnabled = 1;
    m.logging = 1;
    pause(2)
    %% Filter settings
    t0 = [];  % Initial time (initialize on first data received)
    t = 0;
    n = 4;   % Assuming that you use q as state variable.
    % Add your filter settings here.
    T = m.sampleRate;

    % Current filter state.
    x = [1; 0; 0 ;0];
    P = eye(n, n);
    est_x = x;
    est_P = P;

    g = 9.81;
    alpha = 0.02;
    acc_outlier_thresh = 0.05;
    mag_outlier_thresh = 0.1;

    % Saved filter states.
    xhat = struct('t', zeros(1, 0),...
        'x', zeros(n, 0),...
        'P', zeros(n, n, 0));
    
    meas = struct('t', zeros(1, 0),...
        'acc', zeros(3, 0),...
        'gyr', zeros(3, 0),...
        'mag', zeros(3, 0),...
        'orient', zeros(4, 0));

    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    
    
    %% Filter loop
    while 1  % Repeat while data is available
        flag_acc = 0;
        flag_mag = 0;
        flag_gyro = 0;
        t = t + 1/m.SampleRate;  % Extract current time
        counter = 0;  % Used to throttle the displayed frame rate

        gyro = angvellog(m);
        acc = accellog(m);
        mag = magfieldlog(m);
        orientation = deg2rad(orientlog(m));  % Google's orientation estimate.

        N_acc = length(acc);
        N_mag = length(mag);
        N_gyro = length(gyro);
        N_orientation = length(orientation);

        if isempty(t0)  % Initialize t0
            t0 = t;

            gyro_cov = 1.0e-06 *[0.6066   -0.1000   -0.0131;
                                -0.1000    0.7066    0.0245;
                                -0.0131    0.0245    0.9139];
            
            acc_cov = 1.0e-03 * [0.4492   -0.0118   -0.0535;
                                -0.0118    0.5155    0.0438;
                                -0.0535    0.0438    0.7490]; %1.0e-04 *

            mag_cov = 10 * [0.0267   -0.0102   -0.0060;
                          -0.0102    0.0148    0.0036;
                          -0.0060    0.0036    0.0307];

            g0 = [-0.0355; -0.3737; 9.7855];
            m0 = [0; 31.0063; -56.2152];
            
        end

        N = min([N_gyro,N_mag,N_acc,N_orientation]);
        L_hat(1) = norm(mag(1,:));
        for i = 2:N
            if ~isnan(gyro(i,:))
                [est_x, est_P] = tu_qw(est_x, est_P, gyro(i-1,:)', T, gyro_cov);
                [est_x, est_P] = mu_normalizeQ(est_x, est_P);
                flag_gyro = 1;
                disp("Predicted")
            else
                flag_gyro = 0;
            end

            % Accelerometer update
            if ~isnan(acc(i,:)) & flag_gyro == 1
                if abs(norm(acc(i,:)) - g) < acc_outlier_thresh
                    [est_x, est_P] = mu_g(est_x, est_P, acc(i-1,:)', acc_cov, g0);
                    [est_x, est_P] = mu_normalizeQ(est_x, est_P);
                    flag_acc = 1;
                    disp("Updated with accelerometer")
                else
                    disp("Skipping acc update - Outlier detected")
                    flag_acc = 0;
                end
            else
                flag_acc = 0;
            end

            %Magnetometer update
            if ~isnan(mag(i,:)) & flag_gyro == 1 % Mag measurements are available.
                if abs(norm(mag(i-1,:)) - L_hat(i-1)) < mag_outlier_thresh
                    [est_x, est_P] = mu_m(est_x, est_P, mag(i-1,:)', m0, mag_cov);
                    [est_x, est_P] = mu_normalizeQ(est_x, est_P);
                    L_hat(i) = (1-alpha) * L_hat(i-1) + alpha * norm(mag(i,:));
                    disp("Updated with magnetometer")
                    flag_mag = 1;
                else
                    disp('Skipping mag update - Outlier detected')
                    L_hat(i) = (1-alpha) * L_hat(i-1) + alpha * norm(mag(i,:));
                    flag_mag = 0;
                end
            else
                flag_mag = 0;
            end

            if flag_acc == 1 | flag_mag == 1 
                x = est_x';
            else
                x = eul2quat(orientation(i,:))';
            end
            
            % Visualize result
            if rem(counter, 10) == 0
                setOrientation(ownView, x(1:4));
                title(ownView, 'OWN', 'FontSize', 16);
                if ~any(isnan(orientation))
                    if isempty(googleView)
                        subplot(1, 2, 2);
                        % Used for visualization.
                        googleView = OrientationView('Google filter', gca);
                    end
                    setOrientation(googleView, eul2quat(orientation(i,:)));
                    title(googleView, 'GOOGLE', 'FontSize', 16);
                end
            end
            counter = counter + 1;

            % Save estimates
            xhat.x(:, end+1) = x;
            xhat.P(:, :, end+1) = P;
            xhat.t(end+1) = t - t0;

            meas.t(end+1) = t - t0;
            meas.acc(:, end+1) = acc(i,:)';
            meas.gyr(:, end+1) = gyro(i,:)';
            meas.mag(:, end+1) = mag(i,:)';
            meas.orient(:, end+1) = eul2quat(orientation(i,:))';
        end
        acc = [];
        mag = [];
        gyro = [];
        orientation = [];
        discardlogs(m);
        pause(2)
    end
end
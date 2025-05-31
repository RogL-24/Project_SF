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
    counter = 0;  % Used to throttle the displayed frame rate
    
    %% Filter loop
    while 1  % Repeat while data is available
        flag_acc = 0;
        flag_mag = 0;
        flag_gyro = 0;
        t = t + 1/m.SampleRate;  % Extract current time

        if isempty(t0)  % Initialize t0
            t0 = t;
        end

        gyro = angvellog(m);
        gyro_x_mean = mean(gyro(:,1));
        gyro_y_mean = mean(gyro(:,2));
        gyro_z_mean = mean(gyro(:,3));
        gyro_mean = [gyro_x_mean, gyro_y_mean, gyro_z_mean];

        gyro_x_var = var(gyro(:,1));
        gyro_y_var = var(gyro(:,2));
        gyro_z_var = var(gyro(:,3));
        gyro_cov = cov(gyro);

        acc = accellog(m);
        acc_x_mean = mean(acc(:,1));
        acc_y_mean = mean(acc(:,2));
        acc_z_mean = mean(acc(:,3));

        acc_x_var = var(acc(:,1));
        acc_y_var = var(acc(:,2));
        acc_z_var = var(acc(:,3));
        acc_cov = cov(acc);
        g0 = [0;0;acc_z_mean];

        mag = magfieldlog(m);
        mag_x_mean = mean(mag(:,1));
        mag_y_mean = mean(mag(:,2));
        mag_z_mean = mean(mag(:,3));

        mag_x_var = var(mag(:,1));
        mag_y_var = var(mag(:,2));
        mag_z_var = var(mag(:,3));
        mag_cov = cov(mag);

        orientation = orientlog(m);  % Google's orientation estimate.
        N_acc = length(acc);
        N_mag = length(mag);
        N_gyro = length(gyro);

        N = min([N_gyro,N_mag,N_acc]);
        for i = 2:N
            if ~isnan(gyro(i,:))
                [est_x, est_P] = tu_qw(est_x, est_P, gyro(i-1,:)', T, gyro_cov);
                [est_x, est_P] = mu_normalizeQ(est_x, est_P);
                flag_gyro = 1;
            else
                flag_gyro = 0;
            end

            % Accelerometer update
            if ~isnan(acc(i,:)) & flag_gyro == 1
                [est_x, est_P] = mu_g(est_x, est_P, acc(i-1,:)', acc_cov, g0);
                [est_x, est_P] = mu_normalizeQ(est_x, est_P);
                flag_acc = 1;
            else
                flag_acc = 0;
            end

            % Magnetometer update
            if ~isnan(mag(i,:)) & flag_gyro == 1 % Mag measurements are available.
                m0 = [0; sqrt(mag(i-1,1)^2 + mag(i-1,2)^2); mag(i-1,3)];
                [est_x, est_P] = mu_m(est_x, est_P, mag(i-1,:)', m0, mag_cov);
                [est_x, est_P] = mu_normalizeQ(est_x, est_P);
                flag_mag = 1;
            else
                flag_mag = 0;
            end
            
            if flag_acc == 1 | flag_mag == 1 
                x = est_x';
                estimated_orientation(:,i) = q2euler(x');
            else
                x = eul2quat(orientation(i,:))';
                estimated_orientation(:,i) = orientation(i,:);
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
                    setOrientation(googleView, eul2quat(orientation));
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
    end
end
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
    pause(1)
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
        
        gyro = angvellog(m);
        acc = accellog(m);
        mag = magfieldlog(m);
        orientation = deg2rad(orientlog(m));  % Google's orientation estimate.

        N_acc = length(acc);
        N_mag = length(mag);
        N_gyro = length(gyro);

        if isempty(t0)  % Initialize t0
            t0 = t;
            
            [~, gyro_cov] = mean_cov(gyro(:,1), gyro(:,2), gyro(:,3));
            [acc_mean, acc_cov] = mean_cov(acc(:,1), acc(:,2), acc(:,3));
            [mag_mean, mag_cov] = mean_cov(mag(:,1), mag(:,2), mag(:,3));
            g0 = [0;0;acc_mean(3,1)];
            m0 = [0; sqrt(mag_mean(1,1)^2 + mag_mean(2,1)^2); mag_mean(3,1)];
        end
        
        discardlogs(m);

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
                [est_x, est_P] = mu_m(est_x, est_P, mag(i-1,:)', m0, mag_cov);
                [est_x, est_P] = mu_normalizeQ(est_x, est_P);
                flag_mag = 1;
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
        pause(0.5)
    end
end
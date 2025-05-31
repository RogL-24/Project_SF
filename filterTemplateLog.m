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
    % import('com.liu.sensordata.*');  % Used to receive data.
    
    %% Filter settings
    t0 = [];  % Initial time (initialize on first data received)
    t = 0;
    nx = 4;   % Assuming that you use q as state variable.
    % Add your filter settings here.
    
    % Current filter state.
    x = [1; 0; 0 ;0];
    P = eye(nx, nx);
    
    % Saved filter states.
    xhat = struct('t', zeros(1, 0),...
        'x', zeros(nx, 0),...
        'P', zeros(nx, nx, 0));
    
    meas = struct('t', zeros(1, 0),...
        'acc', zeros(3, 0),...
        'gyr', zeros(3, 0),...
        'mag', zeros(3, 0),...
        'orient', zeros(4, 0));
    %% Create data link
    % server = StreamSensorDataReader(3400);
    % Makes sure to resources are returned.
    % sentinel = onCleanup(@() server.stop());
    
    % server.start();  % Start data reception.
    
    % Used for visualization.
    figure(1);
    subplot(1, 2, 1);
    ownView = OrientationView('Own filter', gca);  % Used for visualization.
    googleView = [];
    counter = 0;  % Used to throttle the displayed frame rate.
    
    
    file = load('sensorlog_flat.mat');
    i = 0;
    
    %% Filter loop
    while i <= lenght(file.Acceleration(:,1))  % Repeat while data is available
    
        t = t + 1/100;  % Extract current time
    
        if isempty(t0)  % Initialize t0
            t0 = t;
        end
    
        acc = file.Acceleration(i,1:3)';
        if ~any(isnan(acc))  % Acc measurements are available.
            % Do something
        end
        % gyr = data(1, 5:7)';
        gyr = file.AngularVelocity(i,1:3)';
        if ~any(isnan(gyr))  % Gyro measurements are available.
            % Do something
        end
    
        % mag = data(1, 8:10)';
        file.MagneticField(i,1:3)';
        if ~any(isnan(mag))  % Mag measurements are available.
            % Do something
        end
    
        orientation = data(1, 18:21)';  % Google's orientation estimate.
    
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
                setOrientation(googleView, orientation);
                title(googleView, 'GOOGLE', 'FontSize', 16);
            end
        end
        counter = counter + 1;
    
        % Save estimates
        xhat.x(:, end+1) = x;
        xhat.P(:, :, end+1) = P;
        xhat.t(end+1) = t - t0;
    
        meas.t(end+1) = t - t0;
        meas.acc(:, end+1) = acc;
        meas.gyr(:, end+1) = gyr;
        meas.mag(:, end+1) = mag;
        meas.orient(:, end+1) = orientation;
        i = i + 1;
    end
end
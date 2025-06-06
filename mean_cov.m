%% Sensor Fusion Project - Roger Lokku and Alexander Gustafsson.
function [mean_data, covariance_data] = mean_cov(x_data, y_data, z_data, sensor_no)
    % function to compute mean and cov and histogram plots as well as the
    % plot of the data.
    x_mean = mean(x_data);
    y_mean = mean(y_data);
    z_mean = mean(z_data);
    mean_data = [x_mean; y_mean; z_mean];
    
    covariance_data = cov([x_data, y_data, z_data]);
    nrbin = 50;
    if sensor_no == 1
        sensor = 'Gyro';
        unit = 'rad/s';
        data_title = 'Gyro data';
    elseif sensor_no == 2
        sensor = 'Accelerometer';
        unit = 'm/s^2';
        data_title = 'Acceleration data';
    else
        sensor = 'Magnetometer';
        unit = '\muT';
        data_title = 'Magnetic data';
    end

    figure
    subplot(3,1,1)
    hold on
    grid on
    histogram(x_data, nrbin)
    title(sprintf('%s data distribution in the x-axis',sensor))
    xlabel(unit)
    ylabel('No of data points')
    hold off
    
    subplot(3,1,2)
    hold on
    grid on
    histogram(y_data, nrbin)
    title(sprintf('%s data distribution in the y-axis',sensor))
    xlabel(unit)
    ylabel('No of data points')
    hold off
    
    subplot(3,1,3)
    hold on
    grid on
    histogram(z_data, nrbin)
    title(sprintf('%s data distribution in the z-axis',sensor))
    xlabel(unit)
    ylabel('No of data points')
    hold off

    figure
    hold on
    grid on
    plot(x_data)
    plot(y_data)
    plot(z_data)
    xlabel('0.01s')
    ylabel(unit)
    title(data_title)
    legend('x','y','z')
    hold off
end
function [mean_data, covariance_data] = mean_cov(x_data, y_data, z_data)
    
    x_mean = mean(x_data);
    y_mean = mean(y_data);
    z_mean = mean(z_data);
    mean_data = [x_mean; y_mean; z_mean];
    
    covariance_data = cov([x_data, y_data, z_data]);
end
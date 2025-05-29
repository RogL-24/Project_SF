function [x, P] = tu_qw(x, P, omega, T, Rw)
    % checks if omega is available.
    n = length(omega);
    if n > 0 & ~isnan(omega) 
        % F = I + AT; where A = 1/2 * S(\omega);
        F = eye(length(x)) + 0.5 * Somega(omega) * T;
    else
        F = eye(length(x));
    end

    % G = 1/2 * S(q)*T
    G = 0.5 * Sq(x) * T;

    % State Prediction
    x = F * x + G * mvnrnd(0,Rw)';

    % Covariance Prediction.
    P = F * P * F' + G * Rw * G';
end
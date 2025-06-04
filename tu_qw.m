%% Task 6
function [x, P] = tu_qw(x, P, omega, T, Rw)
    % checks if omega is available.
    n = length(omega);
    if n > 0 && all(~isnan(omega)) 
        % F = I + AT; where A = 1/2 * S(\omega);
        F = eye(length(x)) + 0.5 * T * Somega(omega);
    else
        F = eye(length(x));
    end

    % G = 1/2 * S(q)*T
    G = 0.5 * T * Sq(x);

    % State Prediction
    x = F * x;

    % Covariance Prediction.
    P = F * P * F' + G * Rw * G';
end
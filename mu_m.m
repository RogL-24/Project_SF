function [x, P] = mu_m(x, P, mag, m0, Rm)
    
    h = Qq(x)' * m0; % predicted measurement
    
    % Jacobian
    [H1, H2, H3, H4] = dQqdq(x);
    H(:,1) = H1' * m0;
    H(:,2) = H2' * m0;
    H(:,3) = H3' * m0;
    H(:,4) = H4' * m0;

    % Innovation gain
    S = H * P * H' + Rm;
    %Kalman Gain
    K = P * H' * inv(S);
    % state update 
    x = x + K * (mag - h);
    % Covariance update
    P = P - K * S * K';

end
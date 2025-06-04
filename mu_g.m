%% Task 9
function [x, P] = mu_g(x, P, yacc, Ra, g0)
    
    h = Qq(x)' * g0; % predicted measurement
    
    % Jacobian
    [H1, H2, H3, H4] = dQqdq(x);
    H(:,1) = H1' * g0;
    H(:,2) = H2' * g0;
    H(:,3) = H3' * g0;
    H(:,4) = H4' * g0;

    % Innovation gain
    S = H * P * H' + Ra;
    %Kalman Gain
    if rcond(S) > 1e-15  
        K = P * H' * inv(S);
    else
        K = zeros(4,3);
    end
    % state update 
    x = x + K * (yacc - h);
    % Covariance update
    % P = P - K * S * K';
    P = P - K * H * P;

end
%% LOS guidance law

% By Magnus Dyre-Moe, Patrick Nitschke and Siawash Naqibi

%% Main

function [psi_d, y_int_dot] = integral_los_guidancelaw(x, y, start_point, end_point, delta, y_int)
    % Input :   actual position - x and y
    %           start position for line segment
    %           end position for line segment
    %           lookaheaddistance
    % Returns : Desired course angle
    x_2 = end_point(1);
    y_2 = end_point(2);
    x_1 = start_point(1);
    y_1 = start_point(2);
    
    kappa = 0.2;
    Kp = 1 / delta;
    Ki = kappa * Kp;
    pi_p = atan2(y_2 - y_1, x_2 - x_1);
    
    % Cross-track error
    y_e = -(x-x_1) * sin(pi_p) + (y-y_1) * cos(pi_p);
    
    % Internal state
    y_int_dot = delta * y_e / ( delta^2 + (y_e + kappa*y_int)^2 );
    
    % Desired heading angle
    psi_d = wrapTo2Pi( pi_p - atan(Kp * y_e + Ki * y_int) );
end
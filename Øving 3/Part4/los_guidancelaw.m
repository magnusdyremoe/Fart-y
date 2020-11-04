%% LOS guidance law

% By Magnus Dyre-Moe, Patrick Nitschke and Siawash Naqibi

%% Main

function chi_d = LOSguidancelaw(x, y)
    % Input : actual position
    
    % Retrieve Way points from WP.mat.
    % Will serve as x_2, y_2, x_1 and y_1
    
    y_e = crosstrackWpn(x2, y2, x1, y1, x, y);
    % x_2 and y_2 are straight line end point
    % x_1 and y_1 are straight line strating point.
    % x and y actual position.
    
    pi_p = atan2(y_2 - y_1, x_2 - x_1);
    delta = 10;
    Kp = 1 / delta
    
    chi_d = pi_p - arctan(Kp * y_e)

end
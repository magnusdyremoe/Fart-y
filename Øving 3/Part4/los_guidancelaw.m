%% LOS guidance law

% By Magnus Dyre-Moe, Patrick Nitschke and Siawash Naqibi

%% Main

function chi_d = LOSguidancelaw(x, y)
    % Input : actual position
    
    % Retrieve Way points from WP.mat.
    % Will serve as x_ref and y_ref
    
    flag = 1;
    [x_p, y_p, y_e] = crosstrack(x_t, y_t, x_ref, y_ref, x, y, flag);
    % x_t and y_t target position
    % x_ref and y_ref reference position from WP.mat.
    % x and y actual position.
    
    pi_p = atan2(y_t - y_ref, x_t - x_ref);
    delta = 10;
    Kp = 1 / delta
    
    chi_d = pi_p - arctan(Kp * y_e)

end
%% LOS guidance law

% By Magnus Dyre-Moe, Patrick Nitschke and Siawash Naqibi

%% Main

function chi_d = los_guidancelaw(x, y, start_point, end_point, delta)
    % Input :   actual position - x and y
    %           start position for line segment
    %           end position for line segment
    %           lookaheaddistance
    % Returns : Desired course angle
    x_2 = end_point(1);
    y_2 = end_point(2);
    x_1 = start_point(1);
    y_1 = start_point(2);
    
    %y_e = crosstrackWpt(x_2, y_2, x_1, y_1, x, y);
    
    
    pi_p = atan2(y_2 - y_1, x_2 - x_1);
    Kp = 1 / delta;
    y_e = -(x-x_1) * sin(pi_p) + (y-y_1) * cos(pi_p);
    
    chi_d = ssa( pi_p - atan(Kp * y_e) );
end
h  = 0.05;    % sampling time [s]
N  = 1000;    % no. of samples

% setpoint and controller gains
chi_ref = 10 * pi/180;   % desired course angle


e_max = 15;
a_max = 30;
Kp = e_max/a_max;
Kd = 

zeta_phi = 1; %Design param

Ki = 0 %Can find using root locus
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Define Kp, Ki and Kd using equations (6.5) - (6.9) and 
% (6.12) and (6.13) in Beard & McLain. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% reference model
wn = 0.1;                       % reference model natural frequnecy
xd = [ 0 0 0 ]';                % initial reference model states (3rd order)
z = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%MAIN LOOP

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:N+1
    % 3rd-order reference model for course
    Ad = [ 0 1 0; 0 0 1; -wn^3 -3*wn^2 -3*wn ];
    Bd = [0 0 wn^3]';
    xd_dot = Ad * xd + Bd * chi_ref; 
    

    % error signals (chi and r are measurements)
    e_chi   = ssa(chi-xd(1));                       % course error (rad)
    e_r     = r - xd(2);                                      % yaw rate error (rad/s)

    % course autopilot 
    chi_ref = Kp * e_chi - Kd * e_r + Ki * z;
     
    % Euler integration
    xd = xd + h * xd_dot;
    z_dot = xd(1);
    z = z + h * z_dot;

end
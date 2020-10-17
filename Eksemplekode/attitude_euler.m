% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions. The MSS m-files must be on your
% Matlab path in order to run the script.
%
% System:                      .
%                            Theta = T(Theta)w
%                              .
%                            I w - S(Iw)w = tau
% Control law:                                           ~
%                            tau = -Kd*w - T(Theta)*Kp*Theta
% 
% Definitions:             
%                            I = inertia matrix (3x3)
%                            S(w) = skew-symmetric matrix (3x3)
%                            T(Theta) = transformation matrix (3x3)
%                            tau = control input (3x1)
%                            w = angular velocity vector (3x1)
%                            Theta = Euler angles vector (3x1)
%
% Authors:                   2020-08-20 Thor I. Fossen and Håkon H.Helgesen

%%
clear all; clc;

%% USER INPUTS
h = 0.01;                       % sample time (s)
N  = 10000;                     % number of samples

% model parameters
I = diag([50 40 80]);           % inertia matrix
I_inv = inv(I);

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

% Initial Conditions
phi = 0*deg2rad;            
theta = 0*deg2rad;
psi = 350*deg2rad;
Theta = [phi; theta; psi];
w = deg2rad*[0 0 0]';

% Desired Euler angles
phi_d = 10*deg2rad;
theta_d = 15*deg2rad;
psi_d = -40*deg2rad;
Theta_d = [phi_d; theta_d; psi_d];

% memory allocation
table = zeros(N+1,10);        

% Control gains
K_p = 5*eye(3); 
K_d = 40*eye(3);



%% FOR-END LOOP
for i = 1:N+1
   % time
   t = (i-1)*h;                             
   
   % Kinematics
   [J, J1, J2] = eulerang(phi, theta, psi);
   T_Theta = J2;
   
   % Error in Euler angles (no SSA correction)
   Theta_error = Theta-Theta_d;             
   
   % Control law   
   tau = -K_d*w - T_Theta'*K_p*Theta_error; 
   
   % Euler angles kinematics 
   Theta_dot = T_Theta*w;                   
   
   % Rigid-body kinetics
   w_dot = I_inv*(Smtrx(I*w)*w + tau);      
   
   table(i,:) = [t phi theta psi w' tau'];  % store data in table
   
   Theta = [phi; theta; psi];
   
   % Euler integration
   Theta = Theta + h*Theta_dot;             
   w = w + h*w_dot;               
   
   % Extract current state
   phi = Theta(1);
   theta = Theta(2);
   psi = Theta(3);
end 

%% PLOT FIGURES

t       = table(:,1);  
phi     = rad2deg*table(:,2);
theta   = rad2deg*table(:,3);
psi     = rad2deg*table(:,4);
w       = rad2deg*table(:,5:7);  
tau     = table(:,8:10);

figure(1); clf; grid on;
hold on;
plot(t, phi, 'b');
plot(t, theta, 'r');
plot(t, psi, 'm');
plot(t, phi_d*rad2deg*ones(length(t),1), 'b--');
plot(t, theta_d*rad2deg*ones(length(t),1), 'r--');
plot(t, psi_d*rad2deg*ones(length(t),1), 'm--');
hold off;
legend('phi', 'theta', 'psi', 'FontSize', 20, 'location', 'best');
title('Euler Angles', 'FontSize', 20);
xlabel('time [s]', 'FontSize', 20); ylabel('deg', 'FontSize', 20);

figure(2); clf; grid on;
hold on;
plot(t, w(:,1));
plot(t, w(:,2), 'r');
plot(t, w(:,3), 'm');
hold off;
legend('p', 'q', 'r', 'FontSize', 20);
title('Angular velocity', 'FontSize', 20);
xlabel('time [s]', 'FontSize', 20); ylabel('deg/s', 'FontSize', 20);

figure(3); clf; grid on;
hold on;
plot(t, tau(:,1));
plot(t, tau(:,2), 'r');
plot(t, tau(:,3), 'm');
hold off;
legend('x', 'y', 'z', 'FontSize', 20);
title('Control input - tau', 'FontSize', 20);
xlabel('time [s]', 'FontSize', 20); ylabel('Moments', 'FontSize', 20);
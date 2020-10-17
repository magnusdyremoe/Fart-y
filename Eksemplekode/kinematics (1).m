%% Lecture 1 TTK 4190
% How to simulate position and attitude in Matlab. Simple example.

%% Initial Values

% Euler angles
phi_0 = deg2rad(0);
theta_0 = deg2rad(0);
psi_0 = deg2rad(0);
Theta_0 = [phi_0, theta_0, psi_0];

% Position in NED 
p_0 = zeros(1, 3);

% Body-fixed linear velocity. Assuming forward motion
v_b = [7; 0; 0];

% Body-fixed angular velocity. 
omega_b = [deg2rad(0); deg2rad(0); deg2rad(2)]; % Change third component


%% Simulation parameters and states

% Step length
h = 0.01;

% Number of time steps
N = 5000; 

% Time vector
t = zeros(N,1);

% Allocate matrices for states
Theta = zeros(N,3);
p_n = zeros(N,3);

% Initialize states
Theta(1, :) = Theta_0;
p_n(1, :) = p_0;

%% Main Loop
for i = 1:N-1
   phi = Theta(i, 1); 
   theta = Theta(i, 2);
   psi = Theta(i, 3);
   
   % Find transformation matrix
   [J, J11, J22] = eulerang(phi, theta, psi);
   
   p_dot = J11*v_b;
   Theta_dot = J22*omega_b;
   
   % 1st-order Euler integration
   p_n(i+1,:) = p_n(i,:) + h*p_dot';
   Theta(i+1,:) = Theta(i,:) + h*Theta_dot';
   t(i+1) = t(i) + h;
end

%% Figures
figure(1); clf; close all;
hold on;
plot(t, p_n(:,1), 'LineWidth', 1.5);
plot(t, p_n(:,2), 'r', 'LineWidth', 1.5);
plot(t, p_n(:,3), 'g', 'LineWidth', 1.5);
grid on;
legend('north', 'east', 'down', 'FontSize', 14, 'location', 'best');
xlabel('time [s]', 'FontSize', 14); ylabel('position [m]', 'FontSize', 14);

figure(2); clf;
hold on;
plot(t, rad2deg(Theta(:,1)), 'LineWidth', 1.5);
plot(t, rad2deg(Theta(:,2)), 'r', 'LineWidth', 1.5);
plot(t, rad2deg(Theta(:,3)), 'g', 'LineWidth', 1.5);
grid on;
legend('phi', 'theta', 'psi', 'FontSize', 14, 'location', 'best');
xlabel('time [s]', 'FontSize', 14); ylabel('Angle [deg]', 'FontSize', 14);

figure(3); clf;
plot(p_n(:,2), p_n(:,1), 'LineWidth', 1.5);
grid on;
title('Position in NE plane', 'FontSize', 14);
xlabel('east [s]', 'FontSize', 14); ylabel('north [m]', 'FontSize', 14);
%% Lecture 1 TTK 4190
% How to compute position and attitude in Matlab

%% Initial Values

phi_0 = deg2rad(0);
theta_0 = deg2rad(0);
psi_0 = deg2rad(0);
Theta_0 = [phi_0, theta_0, psi_0];

q_0 = euler2q(phi_0, theta_0, psi_0)';

%Position in NED
p_0 = zeros(1, 3);

% Body-fixed linear velocity
v_b = [7; 0; 0];

% Body-fixed angular velocity
omega_b = [deg2rad(0); deg2rad(0); deg2rad(2)]; 

% Step length
h = 0.01;

% Number of time steps
N = 5000; 

% Time vector
t = zeros(N,1);

% Allocate matrices for states
q = zeros(N,4);
p_n = zeros(N,3);

% Initialize states
q(1, :) = q_0;
p_n(1, :) = p_0;

for i = 1:N-1
   
   % Find transformation matrix
   [J, J11, J22] = quatern(q(i,:)');
   
   p_dot = J11*v_b;
   q_dot = J22*omega_b;
   
   % 1st-order Euler integration
   p_n(i+1,:) = p_n(i,:) + h*p_dot';
   q(i+1,:) = q(i,:) + h*q_dot';
   
   % Normalization procedure
   q(i+1,:) = q(i+1,:)/sqrt(q(i+1,:)*q(i+1,:)');
   
   t(i+1) = t(i) + h;   
end



%% Figures
figure(1); clf;
hold on;
plot(t, p_n(:,1), 'LineWidth', 1.5);
plot(t, p_n(:,2), 'r', 'LineWidth', 1.5);
plot(t, p_n(:,3), 'g', 'LineWidth', 1.5);
grid on;
legend('north', 'east', 'down', 'FontSize', 14);
xlabel('time [s]', 'FontSize', 14); ylabel('position [m]', 'FontSize', 14);

figure(2); clf;
hold on;
plot(t, q(:,1), 'LineWidth', 1.5);
plot(t, q(:,2), 'r', 'LineWidth', 1.5);
plot(t, q(:,3), 'g', 'LineWidth', 1.5);
plot(t, q(:,4), 'k', 'LineWidth', 1.5);
grid on;
legend('q_1', 'q_2', 'q_3', 'q_4', 'FontSize', 14);
xlabel('time [s]', 'FontSize', 14); ylabel('Quaternion', 'FontSize', 14);

figure(3); clf;
plot(p_n(:,2), p_n(:,1), 'LineWidth', 1.5);
grid on;
title('Position in NE plane', 'FontSize', 14)
xlabel('east [s]', 'FontSize', 14); ylabel('north [m]', 'FontSize', 14);

for i = 1:size(q,1)
   [phi(i),theta(i),psi(i)] = q2euler(q(i,:)'); 
end

figure(4); clf;
hold on;
plot(t, rad2deg(phi), 'LineWidth', 1.5);
plot(t, rad2deg(theta), 'r', 'LineWidth', 1.5);
plot(t, rad2deg(psi), 'g', 'LineWidth', 1.5);
grid on;
legend('phi', 'theta', 'psi', 'FontSize', 14);
xlabel('time [s]', 'FontSize', 14); ylabel('Angle [deg]', 'FontSize', 14);


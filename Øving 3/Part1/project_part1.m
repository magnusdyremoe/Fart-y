% Project in TTK4190 Guidance and Control of Vehicles 
%
% Author:           Magnus Dyre-Moe, Patrick Nitschke and Siawash Naqibi
% Study program:    Cybernetics and Robotics

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h  = 0.05;    % sampling time [s]
Ns = 1000;    % no. of samples

psi_ref = 10 * pi/180;  % desired yaw angle (rad)
u_ref   = 7;            % desired surge speed (m/s)
               
% ship parameters 
m = 17.0677e6;          % mass (kg)
Iz = 2.1732e10;         % yaw moment of inertia (kg m^3)
xg = -3.7;              % CG x-ccordinate (m)
L = 161;                % length (m)
B = 21.8;               % beam (m)
T = 8.9;                % draft (m)
KT = 0.7;               % propeller coefficient (-)
Dia = 3.3;              % propeller diameter (m)
rho = 1025;             % density of water (m/s^3)

% rudder limitations
delta_max  = 40 * pi/180;        % max rudder angle      (rad)
Ddelta_max = 5  * pi/180;        % max rudder derivative (rad/s)

% added mass matrix
Xudot = -8.9830e5;
Yvdot = -5.1996e6;
Yrdot =  9.3677e5;
Nvdot =  Yrdot;
Nrdot = -2.4283e10;

% added mass matrix
MA = -[Xudot 0     0;
       0     Yvdot Yrdot;
       0     Nvdot Nrdot]

% rigid-body mass matrix
MRB = [ m 0    0 
        0 m    m*xg
        0 m*xg Iz ];

% Inverted M to use to calculate nu
Minv = inv(MRB+MA);

% Linear damping
T1 = 20;
T2 = 20;
T6 = 10;
Xu = - (m - Xudot)/T1;
Yv = - (m - Yvdot)/T2;
Nr = - (Iz - Nrdot)/T6;

D = diag([-Xu, -Yv, -Nr])

% Constants
k = 0.1;
CR = 0;
epsilon = 0.001;
S = T*L*2 + T*B*2;   %Wettet surface area (m^2)
v = 1e-6;            %Kinematic viscosity (m/s^2)

%Cross flow drag coefficients
Cd_2D = Hoerner(B,T);

% input matrix
t_thr = 0.05;           % thrust deduction number
X_delta2 = 0;           % rudder coefficients (Section 9.5)
Y_delta = 0;      
N_delta = 1;
B = [ (1-t_thr)  X_delta2
        0        Y_delta
        0        N_delta  ];

% initial states
eta = [0 0 0]';
nu  = [0.1 0 0.1]';
delta = 0;
n = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1,14);               % table of simulation data

for i=1:Ns+1

    t = (i-1) * h;                      % time (s)
   
    % state-dependent time-varying matrices
    CRB = m * nu(3) * [ 0 -1 -xg 
                        1  0  0 
                        xg 0  0  ];
    % Added mass coriolis matrix           
    CA = [0                         0           Yvdot*nu(2)+Yrdot*nu(3);
          0                         0           -Xudot*nu(1);
          -Yvdot*nu(2)-Yrdot*nu(3)  Xudot*nu(1) 0];
    
    C = CRB + CA
    
    % Non linear surge damping
    Rn = nu(1)*L/v;
    Cf = 0.075/((log10(Rn) - 2)^2 + epsilon) + CR
    Xn = -0.5*rho*S*(1+k)*Cf*abs(nu(1))*nu(1)
    
    % Cross flow drag
    Yn = 0;
    Nn = 0;
    dx = L/10;
    for xL = -L/2:dx:L/2
        Ucf = abs(nu(2) + xL *nu(3)) * (nu(2) + xL * nu(3));
        Yn = Yn - 0.5 * rho * T * Cd_2D * Ucf * dx; % sway force
        Nn = Nn - 0.5 * rho * T * Cd_2D * xL * Ucf * dx; % yaw moment
    end
    
    R = Rzyx(0,0,eta(3));
    
    % reference models
    psi_d = psi_ref;
    r_d = 0;
    u_d = u_ref;
   
    % thrust 
    thr = rho * Dia^4 * KT * abs(n) * n;    % thrust command (N)
        
    % control law
    delta_c = 0.1;            % rudder angle command (rad)
    n_c = 10;                 % propeller speed (rps)
    
    % ship dynamics
    u = [ thr delta ]';
    tau = B * u;
    nu_dot = Minv * (tau - C * nu - D * nu + [Xn, Yn, Nn]'); 
    eta_dot = R * nu;    
    
    % Rudder saturation and dynamics (Sections 9.5.2)
    if abs(delta_c) >= delta_max
        delta_c = sign(delta_c)*delta_max;
    end
    
    delta_dot = delta_c - delta;
    if abs(delta_dot) >= Ddelta_max
        delta_dot = sign(delta_dot)*Ddelta_max;
    end    
    
    % propeller dynamics
    n_dot = (1/10) * (n_c - n);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [t n_c delta_c n delta eta' nu' u_d psi_d r_d];       
     
    % Euler integration
    eta = euler2(eta_dot,eta,h);
    nu  = euler2(nu_dot,nu,h);
    delta  = euler2(delta_dot,delta,h);   
    n  = euler2(n_dot,n,h);    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t       = simdata(:,1);                 % s
n_c     = 60 * simdata(:,2);            % rpm
delta_c = (180/pi) * simdata(:,3);      % deg
n       = 60 * simdata(:,4);            % rpm
delta   = (180/pi) * simdata(:,5);      % deg
x       = simdata(:,6);                 % m
y       = simdata(:,7);                 % m
psi     = (180/pi) * simdata(:,8);      % deg
u       = simdata(:,9);                 % m/s
v       = simdata(:,10);                % m/s
r       = (180/pi) * simdata(:,11);     % deg/s
u_d     = simdata(:,12);                % m/s
psi_d   = (180/pi) * simdata(:,13);     % deg
r_d     = (180/pi) * simdata(:,14);     % deg/s

figure(1)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions (m)'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi,t,psi_d,'linewidth',2);
title('Actual and desired yaw angles (deg)'); xlabel('time (s)');
subplot(313)
plot(t,r,t,r_d,'linewidth',2);
title('Actual and desired yaw rates (deg/s)'); xlabel('time (s)');


figure(2)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocities (m/s)'); xlabel('time (s)');
subplot(312)
plot(t,n,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed (rpm)'); xlabel('time (s)');
subplot(313)
plot(t,delta,t,delta_c,'linewidth',2);
title('Actual and commanded rudder angles (deg)'); xlabel('time (s)');


% constants
rho = 1025;
g = 9.82;

% box-shaped ship
L = 80;             % Length
B = 20;             % Beam
T = 6;              % Draft
R44 = 0.35 * B;     % radius of gyration
zb = T/2;           % CB
zg = -2;            % CG

% displaced volume and mass
nabla = L * B * T;       
m = rho * nabla;         

% hydrostatics (Section 4.2.3)
I_T = (1/12) * B^3 * L; 
BM_T = I_T/nabla;
BG = zb - zg;
GM_T = BM_T - BG        % transverse metacentric height 

Ix = m * R44^2;         % moment of inertia about the CG
Ix_CF = Ix + m * zg^2;  % moment of inertia about the CF
A44_CF = 0.2 * Ix_CF;   % added moment of inertia (kappa4 = 0.2)

% heave and roll periods in seconds w.r.t. to the CF (Section 4.3.3)
T3 = 2 * pi * sqrt( 2 * T / g)
T4 = 2 * pi * sqrt( (Ix_CF + A44_CF) / (rho * g * nabla * GM_T) )

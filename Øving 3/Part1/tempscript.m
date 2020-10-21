m = 17.0677e6; 
L = 161;                % length (m)
B = 21.8;               % beam (m)

%%Prism              
H_prism = 15.8;
V_prism = L*B*H_prism
rho = m / V_prism
I_zCG = (L^2 + B^2)*m / 12

%%Ship
T_ship = 8.9;
V_ship = L*B*T_ship;
rho = m / V_ship;
I = (L^2 + B^2)*m/12;

r_bg = [-3.7; 0; T/2];

S = Smtrx(r_bg);
S*r_bg

%I_zCO = I_zCG + rho*L^2*B^2;

%%Task2
rho_water = 1025;
nabla = m/rho_water;
A_wp = L*B;
T = nabla/A_wp;
T_3 = 2*pi*sqrt(2*4.7443/9.81);

Iz = 2.1732e10;
I_CO = 3.777e10;
I_CO / Iz

H_prism = 15.8
H_vessel = 8.9
H_prism / H_vessel
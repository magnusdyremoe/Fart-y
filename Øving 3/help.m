L = 161;
B = 21.8;
H = 15.8;
m = 17.0677e6;
w = B;

Iz = 1/12 * m * (L^2 + B^2);
Ix = 1/12 * m * (H^2 + B^2);
Iy = 1/12 * m * (L^2 + H^2);

Ig = diag([Ix, Iy, Iz])

rbg = [-3.7; 0; H/2];
Ibb = Ig + m*((rbg')*rbg*eye(3) - rbg * rbg')

ratio = Ibb(3,3) / Ig(3,3)


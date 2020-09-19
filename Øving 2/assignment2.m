%Assignment 2 Fartøystyring

%Problem 1

V_a = 580;
g = 9.81;
V_g = V_a;


A = [-0.322 0.052   0.028   -1.12   0.002;
    0       0       1       -0.001  0;
    -10.6   0       -2.87   0.46    -0.65;
    6.87    0       -0.04   -0.32   -0.02;
    0       0       0       0       -7.5];

B = [0; 0; 0; 0; 7.5];

C = [1 0 0 0 0;
    0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0];

x = sym('x', [5,1]) 
u = sym('u', [1,1])

%u = aileron_ref * H_l

H_l = tf(7.5, [1, 7.5]) 

course_angle_dot = g/V_g * tan(x(2)) * cos(x(1)) %sideslip = course - heading

x_dot = A*x + B*u
y = C*x


a_phi_1 = 2.87;
a_phi_2 = -0.65;

k_p_phi = -2;
k_d_phi = 1.94;

k_i_phi = 0;

sys = tf([a_phi_2*k_p_phi k_i_phi*a_phi_2] ,[1 a_phi_1+k_p_phi*a_phi_2 a_phi_2*k_p_phi a_phi_2*k_i_phi])

[r, k] = rlocus(sys)
r
k
rlocus(sys)

d = 1.5;
k_p_chi = 3.74;
k_i_chi = 0.21





%Saturation
function aileron_ref = sat(angle)
    if abs(angle) > 30
        aileron_ref = sign(angle)*30
    else
        aileron_ref = angle
    end
end


%% Problem 1.1
%Constants
m = 180;
R_33 = 2.0;
I_g = m*R_33^2 * eye(3);
tau = sym('t', [3,1]);

%State
q = sym('q', [4,1]);
e = q(2:4);
w = sym('w', [3,1]);
x = [e;w];

f1 = Tquat(q)*w;
q_dot = f1(2:4);

Iw = (I_g * w);
S_Iw = [0 -Iw(3) Iw(2) ; 
        Iw(3) 0 -Iw(1) ; 
        -Iw(2) Iw(1) 0 ];
    
w_dot = inv(I_g) * (tau + S_Iw * w);

%Linearization
df1_dx = jacobian(q_dot, x);
df2_dx = jacobian(w_dot, x);

A = [df1_dx; df2_dx];
B = [jacobian(q_dot, tau); 
    jacobian(w_dot, tau)];

%%Problem 1.2
k_d = 40;
k_p = 2;


K_d = k_d * eye(3);

K = [k_p*eye(3) K_d];
tau = -K*x;

%System stability with state feedback given by eig(A-BK)

mat = [eye(3)*0, eye(3)*0.5;
       -eye(3)*k_p/720, -eye(3)*k_d/720]
eig(mat)


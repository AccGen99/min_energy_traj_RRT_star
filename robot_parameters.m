% Initial & Final states
alpha_x = -1;

% Constants
L = 0.1255;             % metres
Fv = 0.0025;            % N.m/(rad/sec)
Kb = 0.0255;            % V/(rad/sec)
Kt = 0.0255;            % N.m/A
Mc = 1.59;              % Kg
Mw = 0.133;             % Kg
Vs = 24;                % Volt
r = 0.03;               % metres
tw = 0.045;             % metres
Ra = 7.9;               % Ohm
n = 1;                  % Assumed Gear Ratio

% Matrices & constants used in calculations
M = Mc + 3*Mw;
Jw = 0.5*Mw*r^2;
Jc = 0.5*Mc*L^2;
J = 3*(Jw + Mw*L^2 + 0.25*Mw*r^2) + Jc;
B = [0, 1, L; -sin(pi/3), -cos(pi/3), L; sin(pi/3), -cos(pi/3), L];
M = diag([M, M, J]);
k2 = Vs*Kt*n/(Ra*r);
D = k2*B';
k1 = 1.5*(Fv+Kt*Kb*n^2/Ra)/r^2;
C = k1*diag([1, 1, 2*L^2]);
Q = B'*B;
A = M+Jw*Q/r^2;
w1 = (Vs^2)/Ra;
w2 = Kb*n*Vs/(Ra*r);

% For differential equation of phi-dot
phi_f = phi(2) - phi(1);
C3 = (C(3,3)/A(3,3))^2 - k2*w2*Q(3,3)*C(3,3)/(w1*A(3,3)^2);
C4 = Q(3,3)*(k2^2)/(2*w1*(A(3,3))^2);
h3 = 3*(L^2)*(Fv+Kt*Kb*(n^2)/Ra)/r^2;
k = (h3^2 - 3*(L^2)*h3*Kt*Kb*(n^2)/(Ra*(r^2)))/(Jc + (3*Jw*L^2)/r^2)^2;
tau_w = 1/(k^0.5);

% For differential equation of x_dot
R = [cos(phi(1)), -sin(phi(1)), 0; sin(phi(1)), cos(phi(1)), 0; 0, 0, 1];
R_dot = [-sin(phi(1)), -cos(phi(1)), 0; cos(phi(1)), ...
    -sin(phi(1)), 0; 0, 0, 0];
C1 = (C(1,1)/A(1,1))^2 - k2*w2*Q(1,1)*C(1,1)/(w1*A(1,1)^2);
C2 = (k2^2)*Q(1,1)/(2*w1*A(1,1)^2);
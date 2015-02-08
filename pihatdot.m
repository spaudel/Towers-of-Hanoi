function output = pihatdot(u )
%Calculate pihatdot for parameter adaptive control
%pihatdot = K_pi*Y'*sigma
global Kp Kd;
theta = [u(1); u(2)];
thetadot = [u(3); u(4)];
theta_r = [u(5); u(6)];
thetadot_r = [u(7); u(8)];
thetadotdot_r = [u(9); u(10)];

%Define K_pi: user defined diagonal adaptive control gain matrix 4x4
k1 = 100;
k2 = 100;
k3 = 100;
k4 = 100;
K_pi = [k1 0 0 0;0 k2 0 0; 0 0 k3 0; 0 0 0 k4];   
Kpi_inv = inv(K_pi);

e = theta_r - theta;
edot = thetadot_r - thetadot;
sigma = edot + inv(Kd)* Kp * e;

Ymatrix = Y(u);

output = Kpi_inv * Ymatrix' * sigma;
end


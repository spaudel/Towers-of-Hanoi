function tau = hanoi_torque( u )
%u = vector of reference link angular positions, velocities, and
%accelerations. calculates required torque for open loop
theta1_r = u(1);
theta2_r = u(2);
theta1dot_r = u(3);
theta2dot_r = u(4);
theta1dotdot_r = u(5);
theta2dotdot_r = u(6);

theta_r = [theta1_r; theta2_r];
thetadot_r = [theta1dot_r; theta2dot_r];
thetadotdot_r = [theta1dotdot_r; theta2dotdot_r];

global m mL a Iz l b1 b2 g; %system parameters
global tf ts omega_n zeta ; % model parameters

%define matrices
m11 = m*(a^2 + 2*l^2 + 2*a*l*cos(theta2_r)) + 2* Iz + 2*mL*a^2*(1+cos(theta2_r));
m12 = m*l*(l+a*cos(theta2_r)) + Iz + mL*a^2*(1+cos(theta2_r));
m21 = m*l*(l+a*cos(theta2_r))+Iz+mL*a^2*(1+cos(theta2_r));
m22 = m*l^2+Iz+mL*a^2;
M_theta = [m11 m12; m21 m22];

b11 = -2*(m*a*l+mL*a^2)*sin(theta2_r)*theta2dot_r;
b12 = -(m*a*l + mL*a^2)*sin(theta2_r)*theta2dot_r;
b21 = (m*a*l + mL*a^2)*sin(theta2_r)*theta1dot_r;
b22 = 0;
b_theta = [b11 b12; b21 b22];

B = [b1 0;0 b2];

g11 = (m*(a+l) + mL*a)*g*cos(theta1_r) + (m*l + mL*a)*g*cos(theta1_r + theta2_r);
g12 = (m*l + mL*a)*g*cos(theta1_r + theta2_r);
g_theta = [g11; g12];

Tau_r = M_theta*thetadotdot_r + b_theta*thetadot_r + B*thetadot_r + g_theta;
tau = Tau_r;

end


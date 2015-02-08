function out = feedback_lin(u)
%u is vector of angles, velocities, accelerations + v (v = Ke). output is
%control signal, "u"
theta1 = u(1);
theta2 = u(2);
theta1dot = u(3);
theta2dot = u(4);
thetadotdot_r_v = [u(5); u(6)]; %represents theta1dotdot_r + v

thetadot = [theta1dot; theta2dot];

global m mL a Iz l b1 b2 g; %system parameters
global tf ts omega_n zeta ; % model parameters

%define matrices
m11 = m*(a^2 + 2*l^2 + 2*a*l*cos(theta2)) + 2* Iz + 2*mL*a^2*(1+cos(theta2));
m12 = m*l*(l+a*cos(theta2)) + Iz + mL*a^2*(1+cos(theta2));
m21 = m*l*(l+a*cos(theta2))+Iz+mL*a^2*(1+cos(theta2));
m22 = m*l^2+Iz+mL*a^2;
M_theta = [m11 m12; m21 m22];

b11 = -2*(m*a*l+mL*a^2)*sin(theta2)*theta2dot;
b12 = -(m*a*l + mL*a^2)*sin(theta2)*theta2dot;
b21 = (m*a*l + mL*a^2)*sin(theta2)*theta1dot;
b22 = 0;
b_theta = [b11 b12; b21 b22];

B = [b1 0;0 b2];

g11 = (m*(a+l) + mL*a)*g*cos(theta1) + (m*l + mL*a)*g*cos(theta1 + theta2);
g12 = (m*l + mL*a)*g*cos(theta1 + theta2);
g_theta = [g11; g12];

control_u = M_theta*thetadotdot_r_v + b_theta*thetadot + B*thetadot + g_theta;
out = control_u;

end


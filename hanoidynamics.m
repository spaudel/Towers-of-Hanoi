function alpha = hanoidynamics(u)
%models dynamics of robot arm. u is vector of applied torques and measured 
%link angular positions and velocities. alpha is thetadotdot

%define inputs
tau1 = u(1);
tau2 = u(2);
theta1 = u(3);
theta2 = u(4);
theta1dot = u(5);
theta2dot = u(6);

%global m mL a Iz l b1 b2 g; %system parameters
global tf ts omega_n zeta ; % model parameters

m_nom = 0.25;
mL_nom = 0.1;
a_nom = 0.2;
Iz_nom= 0.01;
l_nom = a_nom/2;
b1_nom = 0.2;
b2_nom = 0.2;
g = 9.81;

%define matrices
m11 = m_nom*(a_nom^2 + 2*l_nom^2 + 2*a_nom*l_nom*cos(theta2)) + 2* Iz_nom + 2*mL_nom*a_nom^2*(1+cos(theta2));
m12 = m_nom*l_nom*(l_nom+a_nom*cos(theta2)) + Iz_nom + mL_nom*a_nom^2*(1+cos(theta2));
m21 = m_nom*l_nom*(l_nom+a_nom*cos(theta2))+Iz_nom+mL_nom*a_nom^2*(1+cos(theta2));
m22 = m_nom*l_nom^2+Iz_nom+mL_nom*a_nom^2;
M_theta = [m11 m12; m21 m22];

b11 = -2*(m_nom*a_nom*l_nom+mL_nom*a_nom^2)*sin(theta2)*theta2dot;
b12 = -(m_nom*a_nom*l_nom + mL_nom*a_nom^2)*sin(theta2)*theta2dot;
b21 = (m_nom*a_nom*l_nom + mL_nom*a_nom^2)*sin(theta2)*theta1dot;
b22 = 0;
b_theta = [b11 b12; b21 b22];

B = [b1_nom 0;0 b2_nom];

g11 = (m_nom*(a_nom+l_nom) + mL_nom*a_nom)*g*cos(theta1) + (m_nom*l_nom + mL_nom*a_nom)*g*cos(theta1 + theta2);
g12 = (m_nom*l_nom + mL_nom*a_nom)*g*cos(theta1 + theta2);
g_theta = [g11; g12];

Tau = [tau1;tau2];
Theta_dot = [theta1dot; theta2dot];
%solve for thetadotdot
thetadotdot = inv(M_theta) * (Tau - b_theta*Theta_dot - B*Theta_dot - g_theta); 
alpha = thetadotdot;


end
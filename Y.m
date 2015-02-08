function outY = Y(u)
%function to calculate Y for parameter adaptive control
%   Detailed explanation goes here
global m mL mLuser a Iz1 Iz2 Iz l b1 b2 g;
global Kd Kp

theta = [u(1); u(2)];
thetadot = [u(3); u(4)];
theta_r = [u(5); u(6)];
thetadot_r = [u(7); u(8)];
thetadotdot_r = [u(9); u(10)];

e = theta_r - theta;
edot = thetadot_r - thetadot;
Kdinv = inv(Kd);
thetadot_ra = thetadot_r + Kdinv * Kp * e;
thetadotdot_ra = thetadotdot_r + Kdinv * Kp * edot;

%Define cosines
c1 = cos(theta(1));
c2 = cos(theta(2));
c12 = cos(theta(1) + theta(2));

%Define sines
s1 = sin(theta(1));
s2 = sin(theta(2));
s12 = sin(theta(1) + theta(2));

y11 = 2*thetadotdot_ra(1) + thetadotdot_ra(2);
y21 = thetadotdot_ra(1) + thetadotdot_ra(2);
y12 = (a^2 + 2*l^2 + 2 * a * l * c2) *thetadotdot_ra(1) + l*(l+ a*c2)*thetadotdot_ra(2) - a*l*s2*(2*thetadot_ra(1) +thetadot_ra(2)) * thetadot(2) + g*((a+l)*c1 + l * c12);
y22 = l*(l+a*c2)*thetadotdot_ra(1) + l^2 * thetadotdot_ra(2) + a*l*s2* thetadot(1)* thetadot_ra(1) + g*l*c12;
y13 = a^2*(1+c2)*(2*thetadotdot_ra(1) + thetadotdot_ra(2)) - a^2 * s2* (2*thetadot_ra(1) + thetadot_ra(2))*thetadot(2) + g*a *(c1 + c12);
y23 = a^2 * (1+c2) * thetadotdot_ra(1) + a^2*thetadotdot_ra(2) + a^2 * s2 * thetadot(1) * thetadot_ra(1) + g*a*c12;
y14 = thetadot_ra(1);
y24 = thetadot_ra(2);

outY = [y11 y12 y13 y14; y21 y22 y23 y24];

end


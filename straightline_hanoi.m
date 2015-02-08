function p = straightline_hanoi(pi,pf,tf,t)
syms a5 a4 a3;
dist = sqrt((pf(2) - pi(2))^2 + (pf(1)-pi(1))^2);
a3 = 10*dist;
a4 = -15*dist;
a5 = 6*dist;
s_t = a5*(t/tf).^5 + a4*(t/tf).^4 + a3*(t/tf).^3;

theta = atan2(pf(2)-pi(2),pf(1)-pi(1));

x = pi(1) + s_t*cos(theta);
y = pi(2) + s_t * sin(theta);

p = [x;y];
end
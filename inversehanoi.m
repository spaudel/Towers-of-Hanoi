function theta = inversehanoi(px,py)
% calculates joint angle vector position given the end effector position
    a = 0.2; %m
    alpha = atan(py/px);
    beta = acos((px^2 + py^2)/(2*a*sqrt(px^2+py^2)));
    
    theta1 = alpha + beta;
    
    ctheta2 = (px^2 + py^2 - a^2 -a^2)/(2*a*a);
    stheta2 = - sqrt(1 - ctheta2^2); % for elbow up
    
    theta2 = atan2(stheta2, ctheta2);
    
    theta = [theta1; theta2];
end

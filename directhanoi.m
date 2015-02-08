function p = directhanoi(theta)
    %given angle, calculates position of end effector
    a = 0.2;
    t1 = theta(1);
    t2 = theta(2);
    
    p1 = a* cos(t1)+ a* cos(t1 + t2);
    p2 = a* sin(t1) + a* sin(t1 + t2);
    
    p = [p1;p2];
end
function P = Lyapunov_P( Kp, Kd )
%Calculates P that satisfies Lyapunov equation F'P + PF = -I

F = [zeros(2) eye(2); -Kp -Kd];
P = lyap(F,eye(4));

end


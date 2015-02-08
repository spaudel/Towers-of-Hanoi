function p_r = hanoitraj(t)
%sequences all five moves for completion in 5 sec. p_r is vector of px and
%py

global mL mLuser tf; %system parameters

%define the 5 positions
p0 = [20;20]./100;
p1 = [10;-10]./100;
p2 = [10;0]./100;
p3= [30;0]./100;
p4 = [30;-10]./100;
p5 = [20;20]./100;

if(t <= tf)
    p_r = straightline_hanoi(p0,p1,tf,t);
    mL = 0; 
else if(t <= 2*tf)
        p_r = straightline_hanoi(p1,p2,tf,(t-tf)); 
        mL = mLuser;
    else if (t <= 3*tf)
           p_r = straightline_hanoi(p2,p3,tf,(t -2*tf));
    else if (t <= 4*tf)
            p_r = straightline_hanoi(p3,p4,tf,(t-3*tf));
    else if (t <= 5*tf)
            p_r = straightline_hanoi(p4,p5,tf,(t-4*tf));
            mL = 0;
        else 
        p_r = straightline_hanoi(p5,p0,tf,(t-5*tf));
        end
        end
        end
    end
end
end

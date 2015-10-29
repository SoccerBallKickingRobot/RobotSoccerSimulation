function b = RobotLegb(KC,u)

b = zeros(KC.DOF,1);
th1 = KC.states(1);
th2 = KC.states(2);
th3 = KC.states(3);
dth1 = KC.states(4);
dth2 = KC.states(5);
dth3 = KC.states(6);
tau1 = u(1,1);
tau2 = u(2,1);
tau3 = u(3,1);

b(1,1) = tau1 - (981*cos(th1 + th2 + th3))/3200 - (2943*sin(th1 + th2))/800 - (6867*sin(th1))/800 + dth3*((dth1*cos(th2 + th3))/32 + (dth2*cos(th2 + th3))/64 + (dth3*cos(th2 + th3))/64 + (dth1*cos(th3))/32 + (dth2*cos(th3))/32 + (dth3*cos(th3))/64) + dth2*((dth1*cos(th2 + th3))/32 + (dth2*cos(th2 + th3))/64 + (dth3*cos(th2 + th3))/64 + (3*dth1*sin(th2))/8 + (3*dth2*sin(th2))/16);
b(2,1) = tau2 - (981*cos(th1 + th2 + th3))/3200 - (2943*sin(th1 + th2))/800 - (3*dth1^2*sin(th2))/16 + dth2*((dth1*cos(th2 + th3))/64 + (3*dth1*sin(th2))/16) + dth3*((dth1*cos(th2 + th3))/64 + (dth1*cos(th3))/32 + (dth2*cos(th3))/32 + (dth3*cos(th3))/64) - (dth1^2*cos(th2 + th3))/64 - (3*dth1*dth2*sin(th2))/16 - (dth1*dth2*cos(th2 + th3))/64 - (dth1*dth3*cos(th2 + th3))/64;
b(3,1) = tau3 - (981*cos(th1 + th2 + th3))/3200 + dth3*((dth1*cos(th2 + th3))/64 + (dth1*cos(th3))/64 + (dth2*cos(th3))/64) - (dth1^2*cos(th2 + th3))/64 - (dth1^2*cos(th3))/64 - (dth2^2*cos(th3))/64 - (dth1*dth2*cos(th3))/32 - (dth1*dth3*cos(th3))/64 - (dth2*dth3*cos(th3))/64 - (dth1*dth3*cos(th2 + th3))/64;

end


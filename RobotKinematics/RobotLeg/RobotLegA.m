function A = RobotLegA(KC)

A = zeros(KC.DOF,KC.DOF);
th1 = KC.states(1);
th2 = KC.states(2);
th3 = KC.states(3);
dth1 = KC.states(4);
dth2 = KC.states(5);
dth3 = KC.states(6);

A(1,1) = (3*cos(th2))/8 - sin(th2 + th3)/32 - sin(th3)/32 + 451/768;
A(1,2) = (3*cos(th2))/16 - sin(th2 + th3)/64 - sin(th3)/32 + 33/256;
A(1,3) = 1/256 - sin(th3)/64 - sin(th2 + th3)/64;
A(2,1) = (3*cos(th2))/16 - sin(th2 + th3)/64 - sin(th3)/32 + 33/256;
A(2,2) = 163/768 - sin(th3)/32;
A(2,3) = 1/256 - sin(th3)/64;
A(3,1) = 1/256 - sin(th3)/64 - sin(th2 + th3)/64;
A(3,2) = 1/256 - sin(th3)/64;
A(3,3) = 7/768;
end


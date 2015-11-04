function out1 = RobotLegFootVelocity(in1)
%ROBOTLEGFOOTVELOCITY
%    OUT1 = ROBOTLEGFOOTVELOCITY(IN1)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    03-Nov-2015 21:06:16

dth1 = in1(4,:);
dth2 = in1(5,:);
dth3 = in1(6,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = cos(th1);
t3 = cos(th2);
t4 = sin(th1);
t5 = sin(th2);
t6 = pi.*(1.0./8.0);
t7 = t6+th3;
t8 = t2.*t3.*(1.0./8.0);
t9 = cos(t7);
t10 = t2.*t3;
t17 = t4.*t5;
t11 = t10-t17;
t12 = t9.*t11.*(1.0./1.6e1);
t13 = sin(t7);
t14 = t2.*t5;
t15 = t3.*t4;
t16 = t14+t15;
t18 = t2.*t5.*(1.0./8.0);
t19 = t3.*t4.*(1.0./8.0);
t20 = t9.*t16.*(1.0./1.6e1);
t21 = t11.*t13.*(1.0./1.6e1);
out1 = [dth1.*(t2.*(1.0./8.0)+t8+t12-t4.*t5.*(1.0./8.0)-t13.*t16.*(1.0./1.6e1))+dth2.*(t8+t12-t4.*t5.*(1.0./8.0)-t13.*t16.*(1.0./1.6e1))+dth3.*(t12-t13.*t16.*(1.0./1.6e1));0.0;dth3.*(t20+t21)+dth1.*(t4.*(1.0./8.0)+t18+t19+t20+t21)+dth2.*(t18+t19+t20+t21)];

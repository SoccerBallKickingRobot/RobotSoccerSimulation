function Robot = Simulation2(Robot)
%% =================Simulate Solved Optimized Trajectory===================

tspan = [0 25];
z0 = [-pi/4; pi/2; 0; 0];
opts = odeset('AbsTol',1e-8,'RelTol',1e-6);
sol = ode45(@dynamics,tspan,z0,opts,p);
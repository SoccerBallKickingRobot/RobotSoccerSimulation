function f = Objective(X0,z0,Robot,SoccerBall,KKnee,opt)

Robot = Simulation7(X0,z0,Robot,SoccerBall);

KC = Robot.KinematicChains.RL;
% Current foot position, velocity, and Jacobian
th1 = KC.states(1); th2 = KC.states(2); th3 = KC.states(3);
dth1 = KC.states(4); dth2 = KC.states(5); dth3 = KC.states(6);
dx = eval(KC.symbolic.dynamics.dx);
f = -(max(Robot.KinematicChains.RL.traj.y(10,:))^2);

fprintf('Function: %f,   Ball Speed: %f,   Stiffness: %f\n',f,sqrt(-f),X0);
end
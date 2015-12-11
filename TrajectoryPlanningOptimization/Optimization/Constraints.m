function [cineq ceq] = Constraints(X0,z0,Robot,SoccerBall,KKnee,opt)

if (opt == 1)
    Robot = Simulation6(X0,z0,Robot,SoccerBall,KKnee);
    
    t1 = X0(1); t2 = X0(2);
    
    cineq = [0.1 - Robot.KinematicChains.RL.traj.y(1,end), t1-t2];
    ceq = [];
    
elseif (opt == 2)
    Robot = Simulation7(X0,z0,Robot,SoccerBall);
    
    cineq = [-Robot.KinematicChains.RL.traj.y(10,end)];
    ceq = [];
end
end
function traj = TrajectoriesRobotLeg(i, traj)
%% =======================Trajectories Robot Leg===========================
% 2.740: Bio-Inspired Robotics
% Soccer Ball Kicking Robot
% Gerardo Bledt
% October 30, 2015
%
% Creates various trajectories for the RoboHAZMAT robotic system.

if (traj.traj == 1)
    % Creates the vertical figure 8 trajectory
    if (i == 0)
        traj.velocity = 8;  % Set velocity
        traj.runs = 360; % Number of runs
        traj.noise = 0; % No noise
    end
    %i = 6*i;
    traj.point(1,1) = 0.25*sind(1*i);
    traj.point(2,1) = 0;
    if (traj.point(1,1) > 0)
        traj.point(3,1) = 0.05;
    else
        traj.point(3,1) = -0.15*cosd(2*i) + 0.2;
    end
    
    
elseif (traj.traj == 2)
    % Creates a vaguely circular noisy trajectory
    if (i == 0)
        traj.velocity = 8;  % Set velocity
        traj.runs = 360; % Number of runs
        traj.noise = 1; % Adds noise
    end
    traj.point(1,1) = 0.05*cosd(2*i + 180) + 0.35 + NoiseCalc;
    traj.point(2,1) = -0.2*sind(i + 210) - 0.15 + NoiseCalc;
    traj.point(3,1) = 0.2*cosd(i) + 0.3 + NoiseCalc;
end
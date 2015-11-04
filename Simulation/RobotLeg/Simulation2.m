%% =================Simulate Solved Optimized Trajectory===================
function Robot = Simulation2(Robot)
KC = Robot.KinematicChains.RL;

tspan = 0:.01:2*pi;
z0 = zeros(6,1);
opts = odeset('AbsTol',1e-3,'RelTol',1e-3);
[x,y] = ode23t(@dynamics,tspan,z0,opts,KC,Robot);

Robot.KinematicChains.RL.traj.x = x';
Robot.KinematicChains.RL.traj.y = y';
Robot.KinematicChains.RL.traj.KCName = KC.Name;

PlaybackTrajectory(Robot, Robot.KinematicChains.RL.traj, 1)
end

%%
function tau = control_law(t,KC)
t

% Controller gains
K = [100; 0; 100];
D = [5; 0; 5];

% Calculate desired trajectory
traj.traj = 1;
t=t+pi;
traj = TrajectoriesRobotLeg(t*180/pi,traj);
footPosD = traj.point;
%footPosD = [-0.25;0;0.29];
%footPosD = [0.02+t/50;0;0.075];

% Current foot position, velocity, and Jacobian
footPos = KC.points.pG(1:3,3);
dFootPos = RobotLegFootVelocity(KC.states);
Jleg  = RobotLegJfoot(KC.states);

% Calculated control input
tau = -Jleg'*(K.*(footPos - footPosD) + D.*dFootPos);
%tau = [0;0;0];
end

%%
function Fa = angleForce(KC,n,K,D)
C = KC.states(n);
dC = KC.states(n+KC.DOF);
Fa = -K*C - D*dC;
if (C > KC.optimization.bounds.lb(n) && C < KC.optimization.bounds.ub(n))
    Fa = 0;
end
end

%%
function dz = dynamics(t,z,KC,Robot)
KC.states = z;
KC = RotateKinematicChain(KC,z(1:3,1));

%     Robot.KinematicChains.RL = KC;
%     RobotPlot(Robot);
%     drawnow;

% Get mass matrix
A = RobotLegA(KC.states);

% Compute Controls
tau = control_law(t,KC);

% Get b = Q - V(q,qd) - G(q)
b = RobotLegb(KC.states, tau);

% Compute the forces restricting the joint angles
Fah = angleForce(KC,1,10,0.1);
Fak = angleForce(KC,2,100,0.1);
Faa = angleForce(KC,3,100,0.1);
QFa = [Fah;Fak;Faa];

% Solve for qdd.
qdd = A\(b+QFa);
dz = 0*z;
% Form dz
dz(1:3) = z(4:6);
dz(4:6) = qdd;
end
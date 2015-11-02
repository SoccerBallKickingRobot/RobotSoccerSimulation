function Robot = Simulation2(Robot)
%% =================Simulate Solved Optimized Trajectory===================
KC = Robot.KinematicChains.RL;

tspan = [0.001, 2];
z0 = zeros(6,1);
opts = odeset('AbsTol',1e-2,'RelTol',1e-2);
sol = ode45(@dynamics,tspan,z0,opts,KC,Robot);
for i = 1:10:length(sol.x)
    Robot.KinematicChains.RL = RotateKinematicChain(KC,sol.y(1:3,i));
    RobotPlot(Robot);
    drawnow;
end

end

function tau = control_law(t,KC)
% Controller gains
K = [1000; 0; 100];
D = [5; 0; 5];

t

traj.traj = 1;
traj = TrajectoriesRobotLeg(t,traj);
footPosD = traj.point;
footPosD = [-0.2;0;0.15];

footPos = KC.points.pG(1:3,3);
dFootPos = RobotLegFootVelocity(KC.states);

Jleg  = RobotLegJfoot(KC.states);

tau = -Jleg'*(K.*(footPos - footPosD) + D.*dFootPos);
footPos - footPosD;
end

function Fa = angleForce(KC,n,K,D)
%% Fixed parameters for contact
C = KC.states(n);
dC = KC.states(n+KC.DOF);
Fa = -K*C - D*dC;
if (C > KC.optimization.bounds.lb(n) && C < KC.optimization.bounds.ub(n))
    Fa = 0;
end
end

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

% Compute the contact force (used for problem 2)
Fah = angleForce(KC,1,10,0.1);
Fak = angleForce(KC,2,100,0.1);
Faa = angleForce(KC,3,100,0.1);

% Compute the contribution of the contact force to the generalied force
% QFc= [0 ; Fc + Fa];  %% YOUR CODE HERE for Q2.2
QFc = [Fah;Fak;Faa];%Fc];

% Solve for qdd.
qdd = A\(b+QFc);
dz = 0*z;
% Form dz
dz(1:3) = z(4:6);
dz(4:6) = qdd;
end
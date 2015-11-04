%% =================Simulate Solved Optimized Trajectory===================
% 2.740: Bio-Inspired Robotics
% Soccer Ball Kicking Robot
% Gerardo Bledt
% November 3, 2015
%
% Simulates the dynamic swinging leg kicking the soccer ball with a very
% rough contact model.
function Robot = Simulation3(Robot)
KC = Robot.KinematicChains.RL;
SoccerBall = CreateSoccerBall;

tspan = 0:.03:pi/3+0.04;
z0 = zeros(12,1);
z0(7:9) = [SoccerBall.dims.radius + 0.05; 0; SoccerBall.dims.radius];
z0(12) = 0;
opts = odeset('AbsTol',1e-3,'RelTol',1e-3);
[x,y] = ode23t(@dynamics,tspan,z0,opts,KC,SoccerBall,Robot);

Robot.KinematicChains.RL.traj.x = x';
Robot.KinematicChains.RL.traj.y = y';
Robot.KinematicChains.RL.traj.KCName = KC.Name;

PlaybackTrajectory(Robot, Robot.KinematicChains.RL.traj, 1)
end

%%
function tau = control_law(t,KC)
%fprintf('%f\n',t);

% Controller gains
K = [100; 0; 100];
D = [5; 0; 5];

% Calculate desired trajectory
traj.traj = 1;
t=t+pi;
traj = TrajectoriesRobotLeg(6*t*180/pi + 180,traj);
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
function Fc = contactForce(Ball,K,D)
C = Ball.states(3) - Ball.dims.radius;
dC = Ball.states(6);
Fc = -K*C - D*dC;
if (C > 0 || Fc < 0)
    Fc = 0;
end
end

%%
function Fk = kickForce(KC,Ball)
footPos = KC.points.pG(1:3,3);
dFootPos = RobotLegFootVelocity(KC.states);
C = Ball.dims.radius - norm(footPos - Ball.states(1:3));
dC = dFootPos(1) - Ball.states(4);
Fk = -Ball.props.K*C - Ball.props.B*dC;
if (C < 0 )
    Fk = 0;
end

dFootPos = RobotLegFootVelocity(KC.states);
if (norm(dFootPos) ~= 0)
    c = dFootPos/norm(dFootPos);
else
    c = zeros(3,1);
end

Fk = Fk*c;

end

%%
function dz = dynamics(t,z,KC,SoccerBall,Robot)
KC.states = z(1:6,1);
SoccerBall.states = z(7:12,1);


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


Fk = kickForce(KC,SoccerBall);
dz(7:9) = z(10:12);
dz(10) = -Fk(1)/SoccerBall.mass;
dz(12) = -9.81 + (contactForce(SoccerBall,10000,10) - Fk(3))/SoccerBall.mass;
end
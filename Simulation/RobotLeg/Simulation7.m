%% =================Simulate Solved Optimized Trajectory===================
% 2.740: Bio-Inspired Robotics
% Soccer Ball Kicking Robot
% Gerardo Bledt
% November 3, 2015
%
% Simulates the dynamic swinging leg kicking the soccer ball with a very
% rough contact model.
function Robot = Simulation7(X0,v,z0,Robot,SoccerBall)
KC = Robot.KinematicChains.RL;

KKnee = X0(1);%5-X0(1);
extraT = 0.0;
tspan = 0:(3*pi/2/v + extraT)/45:(3*pi/2/v + extraT);

SoccerBall.dims.radius
opts = odeset('AbsTol',1e-3,'RelTol',1e-3);
[x,y] = ode23t(@dynamics,tspan,z0,opts,KC,SoccerBall,Robot,KKnee,v);

Robot.KinematicChains.RL.traj.x = x;
Robot.KinematicChains.RL.traj.y = y;
Robot.KinematicChains.RL.traj.KCName = KC.Name;

PlaybackTrajectory(Robot, Robot.KinematicChains.RL.traj, 1)
end

%%
function tau = control_law(t,KC,v)
fprintf('%f\n',t);

% Controller gains
K = [100; 0; 100];
D = [5; 0; 5];

% Calculate desired trajectory
%tc = linspace(0,tf,length(traj)/2);
kneePosD(1,1) = KC.dists.length.l(1)*cos(v*t + pi/2);
kneePosD(2,1) = 0;
kneePosD(3,1) = KC.dists.length.l(1)*(0.3*cos(2*(v*t + pi/2)) - 0.7) + KC.dists.length.hO1;
if (t >= 3*pi/2/v)
    tf = 3*pi/2/v;
    kneePosD(1,1) = KC.dists.length.l(1)*cos(v*tf + pi/2);
    kneePosD(2,1) = 0;
    kneePosD(3,1) = KC.dists.length.l(1)*(0.3*cos(2*(v*tf + pi/2)) - 0.7) + KC.dists.length.hO1;
end

% Current foot position, velocity, and Jacobian
th1 = KC.states(1); th2 = KC.states(2); th3 = KC.states(3);
dth1 = KC.states(4); dth2 = KC.states(5); dth3 = KC.states(6);
kneePos = KC.points.pG(1:3,1);
dx = eval(KC.symbolic.dynamics.dx);
J  = eval(KC.symbolic.dynamics.Jacobian{1});

% Calculated control input
tau = -J'*(K.*(kneePos - kneePosD) + D.*dx(1:3,1));
end

%%
function Fa = angleForce(KC,n,K,D)
C = KC.states(n);                 % Angle
dC = KC.states(n+KC.DOF);         % Angular Velocity
Fa = -K*C - D*dC;                 % Control Law

% Lower and upper angles to constrain the DOF
if (C > KC.optimization.bounds.lb(n) && C < KC.optimization.bounds.ub(n))
    Fa = 0;                       % No force needed if within limits
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
z = KC.states;
th1 = z(1); th2 = z(2); th3 = z(3);
dth1 = z(4); dth2 = z(5); dth3 = z(6);

footPos = KC.points.pG(1:3,3);
dx = eval(KC.symbolic.dynamics.dx);
dFootPos = dx(1:3,3);
C = Ball.dims.radius - norm(footPos - Ball.states(1:3));
dC = dFootPos(1) - Ball.states(4);
Fk = -Ball.props.K*C - Ball.props.B*dC;
if (C < 0 )
    Fk = 0;
end

if (norm(dFootPos) ~= 0)
    c = dFootPos/norm(dFootPos);
else
    c = zeros(3,1);
end

Fk = Fk*c;

end

%%
function FKneeStiff = Stiffness(KC,KKnee)
states = KC.states;
FKneeStiff = -KKnee*states(2);
end

%%
function dz = dynamics(t,z,KC,SoccerBall,Robot,KKnee,v)
KC.states = z(1:6,1);
SoccerBall.states = z(7:12,1);

KC = RotateKinematicChain(KC,z(1:3,1));

%     Robot.KinematicChains.RL = KC;
%     RobotPlot(Robot);
%     drawnow;

th1 = z(1); th2 = z(2); th3 = z(3);
dth1 = z(4); dth2 = z(5); dth3 = z(6);

% Get mass matrix
A = eval(KC.symbolic.dynamics.A);

% Compute Controls
tau = control_law(t,KC,v);
tau1 = tau(1); tau2 = tau(2); tau3 = tau(3);

Fk = kickForce(KC,SoccerBall);
F1 = Fk(1)*0.3; F2 = Fk(2)*0.3; F3 = Fk(3)*0.3; 

% Get b = Q - V(q,qd) - G(q)
b = eval(KC.symbolic.dynamics.b);

% Compute the forces restricting the joint angles
Fah = angleForce(KC,1,10,0.1);
Fak = angleForce(KC,2,100,0.1);
Faa = angleForce(KC,3,100,0.1);

% Kknee function for force
FKneeStiff = Stiffness(KC,KKnee);

QFa = [Fah; Fak + FKneeStiff; Faa];

% Solve for qdd.
qdd = A\(b+QFa);
dz = 0*z;
% Form dz
dz(1:3) = z(4:6);
dz(4:6) = qdd;

dz(7:9) = z(10:12);
dz(10) = (-Fk(1) - 1/2*6*pi*SoccerBall.dims.radius^2*z(10)^2)/SoccerBall.mass;
dz(12) = -9.81 + (contactForce(SoccerBall,10000,10) - Fk(3))/SoccerBall.mass;
end
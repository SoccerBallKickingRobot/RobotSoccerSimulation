function Robot = OptimizeKick(Robot)

KC = Robot.KinematicChains.RL;
SoccerBall = CreateSoccerBall;

opt = 2;
nodes = 2;

z0 = zeros(12,1);
z0(7:9) = [SoccerBall.dims.radius+0.025; 0; SoccerBall.dims.radius];

t1 = 0.25;
t2 = 0.5;
traj = repmat(zeros(2,1)',1,nodes);
traj(1:4) = [-0.1,0.25,0.1,0.25];
lb = repmat([-0.3,-0.0001],1,nodes);
ub = repmat([0.3,1.3],1,nodes);

KKnee = 0.25;

% setup and solve nonlinear programming problem
problem.objective = @(x) Objective(x,z0,Robot,SoccerBall, KKnee, opt);     % create anonymous function that returns objective
problem.nonlcon = @(x) Constraints(x,z0,Robot,SoccerBall, KKnee, opt);     % create anonymous function that returns nonlinear constraints
problem.X0 = [t1,t2,traj];                   % initial guess for decision variables

problem.lb = [0.1,0.1,lb];     % lower bound on decision variables
problem.ub = [1.1,1.1,ub];      % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter','TolCon',1e-2,'TolFun',1e-2,'TolX',1e-2);   % set options
problem.solver = 'fmincon';                     % required
X = fmincon(problem);  


Robot = Simulation6(X,z0,Robot,SoccerBall,KKnee);
PlaybackTrajectory(Robot, Robot.KinematicChains.RL.traj, 1)
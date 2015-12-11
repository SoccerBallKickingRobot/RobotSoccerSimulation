function Robot = OptimizeKneeStiffness(Robot)

KC = Robot.KinematicChains.RL;
SoccerBall = CreateSoccerBall;

z0 = zeros(12,1);
z0(7:9) = [SoccerBall.dims.radius+0.025; 0; SoccerBall.dims.radius];

tf = 0.5;
KKnee = 1.5;  % 0.178853593234190 % 0.23
lb = 0;
ub = 5;

opt = 2;

% setup and solve nonlinear programming problem
problem.objective = @(x) Objective(x,z0,Robot,SoccerBall, KKnee, opt);     % create anonymous function that returns objective
problem.nonlcon = @(x) Constraints(x,z0,Robot,SoccerBall, KKnee, opt);     % create anonymous function that returns nonlinear constraints
problem.X0 = [KKnee];                   % initial guess for decision variables

problem.lb = lb;     % lower bound on decision variables
problem.ub = ub;      % upper bound on decision variables
problem.Aineq = []; problem.bineq = [];         % no linear inequality constraints
problem.Aeq = []; problem.beq = [];             % no linear equality constraints
problem.options = optimset('Display','iter');%,'TolCon',1e-4,'TolFun',1e-4,'TolX',1e-4);   % set options
problem.solver = 'fmincon';                     % required
X = fmincon(problem);  
fprintf('Stiffness: %f\n',X);

Robot = Simulation7(X,z0,Robot,SoccerBall);
PlaybackTrajectory(Robot, Robot.KinematicChains.RL.traj, 1)
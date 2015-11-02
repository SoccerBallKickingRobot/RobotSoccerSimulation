function RLK = RobotLegKinematics
%% =========================Robot Leg Kinematics===========================
% 2.740: Bio-Inspired Robotics
% Soccer Ball Kicking Robot
% Gerardo Bledt
% October 28, 2015
%
% Setup of all physical parameters and the kinematic relationship between
% them. Numerically solves for the Homogeneous transformation based on the
% inputed Denavit-Hartenberg convention as determined for the robot's leg.

%% ============================Numerical Setup=============================
RL = struct();
RL.Name = 'Robot Leg';
RL.Field = 'RLK';
% Number of Degrees of Freedom
RL.DOF = 3;

% Length of the links
RL.d.length = struct();
% Ground origin to hip
RL.d.length.hO1 = 0.25;
% Hip to knee: Thigh
RL.d.length.l(1) = 0.25/2;
% Knee to ankle
RL.d.length.l(2) = 0.25/2;
% Ankle to toe
RL.d.length.l(3) = 0.0625; thAnkle = pi/2; 

% Length of the link center of mass
RL.d.dCOM = struct();
% Thigh COM
RL.d.dCOM.lc(1) = RL.d.length.l(1)/2;
% Knee to ankle
RL.d.dCOM.lc(2) = RL.d.length.l(2)/2;
% Ankle to toe
RL.d.dCOM.lc(3) = RL.d.length.l(3)/2;

% Mass of the links
RL.mass = struct();
RL.mass = [0.09;0.09;0.09/2.2];

% Inertia of the links
RL.inertia = {};
for i = 1:RL.DOF
   RL.inertia{i} = 0*[0, 0, 0;...
       0, (RL.mass(i)*RL.d.length.l(i)^2)/3, 0;...
       0, 0, (RL.mass(i)*RL.d.length.l(i)^2)/3];
end

% Frame points in each frame
RL.pts = struct();
% Origin points placeholder
RL.pts.o = [];
% Joint points
RL.pts.p = [zeros(3,RL.DOF);ones(1,RL.DOF)];
% Defnining new kinematic points
kP = [];
kP = [RL.d.dCOM.lc(1),RL.d.dCOM.lc(2),RL.d.dCOM.lc(3);...
    0, 0, 0;...
    0, 0, 0;...
    1, 1, 1];
% Kinematic points, can add points other than joints
RL.pts.kP = [RL.pts.p,kP];
% Frames for each of the kinematic points first n = DOF are joints
RL.pts.frames = [1;2;3;1;2;3];

% Redefining origin points
RL.pts.o = [0;
    0;
    RL.d.length.hO1;
    1];

% Physical system constraints, upper and lower bounds
RL.opt = struct();
RL.opt.bounds = struct();
RL.opt.bounds.lb = [-pi/2, -170*pi/180, 0];
RL.opt.bounds.ub = [2*pi/3, 0, 0];

% Weighting on importance of points
RL.opt.weightings = zeros(1,size(RL.pts.kP,2));

% Theta Angles
RL.th = struct();

% Theta Angle Definitions
RL.th.thDef = ['Hip Pitch  ';'Knee Pitch '; 'Ankle Pitch'];

% Initial Thetas
RL.th.thi = zeros(RL.DOF,1);
RL.th.thi = [0; 0; thAnkle];
RL.th.dthi = zeros(RL.DOF,1);

%% ========================Mathematical Modeling===========================
% DH Convention
RL.DH = struct();
RL.DH.alphas = [0; 0; 0];
RL.DH.thetas = RL.th.thi;
RL.DH.disps = [0; 0; 0];
RL.DH.offsets = [-RL.d.length.l(1); -RL.d.length.l(2); -RL.d.length.l(3)];

% Homogeneous transformations
RL.DH.H = double(DHTransforms(RL.DH));
RL.DH.HGo = [ 0, -1, 0,     0;
    0, 0, 1,     0;
    1, 0, 0, RL.d.length.hO1;
    0, 0, 0,     1];

RL.states = [RL.th.thi;RL.th.dthi];

%% =====================Create Symbolic Definitions========================
RL.symbs = struct();
RL.symbs.thiSym = sym(zeros(RL.DOF,1));
RL.symbs.alphasSym = sym(zeros(RL.DOF,1));
RL.symbs.thetasSym = sym('th',[1,RL.DOF],'real')';
RL.symbs.dispsSym = sym(zeros(RL.DOF,1));
RL.symbs.offsetsSym = -sym('l',[1,RL.DOF],'real')';
for i = 1:RL.DOF
    RL.symbs.alphasSym(i) = RL.DH.alphas(i);
    RL.symbs.thiSym(i) = RL.th.thi(i);
end

%% ===============Transform each point in the global frame=================
RLK = RotateKinematicChain(KinematicSystem(RL), zeros(RL.DOF, 1));
EOMSymNum(RLK);
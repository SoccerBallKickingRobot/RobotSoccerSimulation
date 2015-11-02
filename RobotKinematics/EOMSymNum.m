function [H, HT, pointsG, dx, T, V, eom, A, b, Jfoot] = ...
    EOMSymNum(KC, request)
%% ====================Equations of Motion Symbolic========================
% 2.740: Bio-Inspired Robotics
% Soccer Ball Kicking Robot
% Gerardo Bledt
% October 28, 2015
%
% Symbolically solves for the Equations of Motion (EOM) of the Kinematic 
% chain passed in. Finds each of the points in the global frame.
name = 'RobotLeg';

%%
DOF = KC.DOF;
% Set up symbolic variables
th =  sym('th',[1,DOF],'real')';
dth =  sym('dth',[1,DOF],'real')';
ddth =  sym('ddth',[1,DOF],'real')';
tau =  sym('tau',[1,DOF],'real')';

q   = th;        % generalized coordinates
dq  = dth;     % first time derivatives
ddq = ddth;  % second time derivatives
u   = tau;
g = 9.81;

%%
ddt = @(r) jacobian(r,[q;dq])*[dq;ddq]; 

% F2Q calculates the contribution of a force to all generalized forces
% for forces, F is the force vector and r is the position vector of the 
% point of force application
F2Q = @(F,r) simplify(jacobian(r,q)'*(F)); 

% M2Q calculates the contribution of a moment to all generalized forces
% M is the moment vector and w is the angular velocity vector of the
% body on which the moment acts
M2Q = @(M,w) simplify(jacobian(w,dq)'*(M));

% Define fundamental unit vectors.  The first element should be the
% horizontal (+x cartesian) component, the second should be the vertical (+y
% cartesian) component, and the third should be right-handed orthogonal.
ihat = [1; 0; 0];
jhat = [0; 1; 0];
khat = cross(ihat,jhat);

%%
DH = struct();
DH.thetas = KC.symbolic.thetasSym + KC.symbolic.thiSym;
DH.alphas = KC.symbolic.alphasSym;
DH.disps = KC.DHParams.disps;
DH.offsets = KC.DHParams.offsets;

HT = DHTransformsSym(DH);

% Global transformations for each DOF
H(:,:,1) = KC.DHParams.HGo*HT(:,:,1);
for i = 2:DOF
    H(:,:,i) = H(:,:,i-1)*HT(:,:,i);
end

% Calculation of the points in global frame
points = KC.points.kP;
pointsG = sym(zeros(4,DOF));
for i = 1:size(KC.points.kP,2)
    pointsG(:,i) = H(:,:,KC.points.frames(i))*points(:,i);
end

% Take time derivatives of vectors as required for kinetic energy terms.
for i = 1:size(KC.points.kP,2)
    dx(:,i) = ddt(pointsG(:,i));
end

Ti = sym(zeros(DOF,1));
omega = sym(zeros(3,DOF));
for i = 1:DOF
    omega(:,i) = [0;0;dq(i)];
    Ti(i,:) = (1/2)*KC.mass(i)*dot(dx(1:3,DOF + i), dx(1:3,DOF + i)) +...
        (1/2)*omega(:,i)'*KC.inertia{i}*omega(:,i);
end
T = simplify(sum(Ti));


Vi = sym(zeros(DOF,1));
for i = 1:DOF
    Vi(i,:) = KC.mass(i)*g*dot(pointsG(1:3,DOF + i), khat);
end
V = simplify(sum(Vi));

Qtau1 = M2Q(tau(1)*jhat, dth(1)*jhat);
Qtau2 = M2Q(tau(2)*jhat, dth(2)*jhat);
Qtau3 = M2Q(tau(3)*jhat, dth(3)*jhat);

Q = Qtau1 + Qtau2 + Qtau3;
%%
E = T+V;                                         % total system energy
L = T-V;                                         % the Lagrangian
eom = ddt(jacobian(L,dq)') - jacobian(L,q)' - Q;

A = jacobian(eom,ddq);
b = A*ddq - eom;


% Compute foot jacobian
Jfoot = jacobian(pointsG(1:3,3),q);

%%% Write functions to evaluate dynamics, etc...
z = sym(zeros(length([q;dq]),1)); % initialize the state vector
z(1:DOF,1) = q;  
z(DOF+1:DOF*2,1) = dq;

directory = 'RobotKinematics/RobotLeg/';
matlabFunction(A,'file',[directory name 'A' ],'vars',{z});
matlabFunction(b,'file',[directory name 'b' ],'vars',{z,u});
matlabFunction(Jfoot,'file',[directory name 'Jfoot' ],'vars',{z});
matlabFunction(dx(1:3,3),'file',[directory name 'FootVelocity' ],'vars',{z});


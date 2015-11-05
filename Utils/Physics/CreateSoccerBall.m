function SoccerBall = CreateSoccerBall
% Create the obj struct and global parameters
obj = struct();
obj.name = 'Soccer Ball';
obj.type = 'Ball';
obj.states = zeros(6,1);
obj.mass = 0.05;

% Create the arrays that describe the sphere
n = 10;
[x,y,z] = sphere(n);
C(:,:,1) = 0.8*ones(n + 1);
C(:,:,2) = 0.8*ones(n + 1);
C(:,:,3) = 0.8*ones(n + 1);
obj.color = struct();
obj.color.name = 'grey';
obj.color.values = C;
obj.dims = struct();
obj.dims.radius = 0.05;
obj.dims.x = x;
obj.dims.y = y;
obj.dims.z = z;

% Calculate the inertia for a hollow sphere
obj.inertia = 2/3*obj.mass*obj.dims.radius^2*eye(3);

% Define the spring constant and damping for the ball to be used as contact
obj.props = struct();
obj.props.K = 1000;
obj.props.B = 0.1;

% Create the SoccerBall Object
SoccerBall = Object(obj);



function SoccerBall = CreateSoccerBall

obj = struct();
obj.name = 'Soccer Ball';
obj.type = 'Ball';
obj.states = zeros(6,1);
obj.mass = 0.1;

n = 10;
[x,y,z] = sphere(n);
C(:,:,1) = 0.8*ones(n + 1);
C(:,:,2) = 0.8*ones(n + 1);
C(:,:,3) = 0.8*ones(n + 1);
obj.color = struct();
obj.color.name = 'grey';
obj.color.values = C;
obj.dims = struct();
obj.dims.radius = 0.08;
obj.dims.x = x;
obj.dims.y = y;
obj.dims.z = z;

obj.inertia = 2/3*obj.mass*obj.dims.radius^2*eye(3);

obj.props = struct();
obj.props.K = 1000;
obj.props.B = 0.1;

SoccerBall = Object(obj);



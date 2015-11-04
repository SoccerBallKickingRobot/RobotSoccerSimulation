function PlaybackTrajectory(Robot, traj, velocity)

if (nargin < 3)
    velocity = 1;
end

fields = fieldnames(Robot.KinematicChains);
for i = 1:length(fields)
    if (strcmpi(traj.KCName,Robot.KinematicChains.(fields{i}).Name))
        field = i;
    end
end

% History vectors for the desired trajectories
histD = zeros(length(traj.x),3);

% History vectors for the actual trajectories
histT = zeros(length(traj.x),3);

for i = 1:velocity:length(traj.x)
    Robot.KinematicChains.(fields{field}) = ...
        RotateKinematicChain(Robot.KinematicChains.(fields{field}),traj.y(1:3,i));
    RobotPlot(Robot);
    CreateSoccerBall(0.15/2,0,0.15/2,0.12/2)
    
    histT(i,:) = Robot.KinematicChains.(fields{field}).points.kPG(1:3,3)';
    
tra.traj = 1;
t=i+pi;
tra = TrajectoriesRobotLeg(t*180/pi,tra);
    
    histD(i,:) = tra.point;
    MS = 10;
    %plot3(histD(:,1),histD(:,2),histD(:,3),'Marker','.','color','green',...
    %    'MarkerSize',MS,'LineStyle','none');
    if(velocity < 5)
    plot3(histT(:,1),histT(:,2),histT(:,3),'Marker','.','color','red',...
        'MarkerSize',MS,'LineStyle','none');
    end
    drawnow;
end
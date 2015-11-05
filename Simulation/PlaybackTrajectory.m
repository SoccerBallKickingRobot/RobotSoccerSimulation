function PlaybackTrajectory(Robot, traj, velocity, opt)

if (nargin < 3)
    velocity = 1;
elseif (nargin < 4)
    opt = 0;
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

SoccerBall = CreateSoccerBall;
firstI = 0;
for i = 1:velocity:length(traj.x)
    Robot.KinematicChains.(fields{field}) = ...
        RotateKinematicChain(Robot.KinematicChains.(fields{field}),traj.y(1:3,i));
    RobotPlot(Robot);
    if (size(traj.y,1) > 6)
        SoccerBall.states = traj.y(7:12,i);
        DrawBall(SoccerBall)
    end
    histT(i,:) = Robot.KinematicChains.(fields{field}).points.kPG(1:3,3)';
    
    tra.traj = 1;
    t=i+pi;
    tra = TrajectoriesRobotLeg(t*180/pi,tra);
    
    histD(i,:) = tra.point;
    MS = 10;
    if(opt == 1 || opt == 3)
        plot3(histT(:,1),histT(:,2),histT(:,3),'Marker','.','color','red',...
            'MarkerSize',MS,'LineStyle','none');
        if (traj.y(10,i) > 0)
            if (firstI == 0)
                firstI = i;
            end
            plot3(traj.y(7,firstI:i),traj.y(8,firstI:i),traj.y(9,firstI:i));
        end
    end
    if(opt == 2 || opt == 3)
        plot3(histD(:,1),histD(:,2),histD(:,3),'Marker','.','color','green',...
            'MarkerSize',MS,'LineStyle','none')
    end
    drawnow;
end
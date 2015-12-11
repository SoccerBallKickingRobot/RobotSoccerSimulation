FigureSetup(RobotLeg);RobotPlot(RobotLeg);
 SoccerBall.states = trajK0.y(1,7:12)';DrawBall(SoccerBall)
% PlaybackTrajectory(RobotLeg, trajK0, 1,1);
% 
% pause(3)
az = 0; el = 9; view([az,el])
pause(1)
RobotPlot(RobotLeg);PlaybackTrajectory(RobotLeg, trajK1, 1,1);

pause(3)

% RobotPlot(RobotLeg);PlaybackTrajectory(RobotLeg, trajK1, 1,1);
% pause(3)
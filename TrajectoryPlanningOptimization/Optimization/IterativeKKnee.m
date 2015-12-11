RobotLeg = CreateRobotLeg;
KC = RobotLeg.KinematicChains.RL;
SoccerBall = CreateSoccerBall;

z0 = zeros(12,1);
z0(7:9) = [SoccerBall.dims.radius+0.05; 0; SoccerBall.dims.radius];

v = 5;

step = 0.04;
ti = 0;
tf = 0.6;

KKneeHist = [ti:step:tf]';
BallXSpeed = zeros(length(KKneeHist),1);

i = 0;
for KKnee = ti:step:tf
    i = i + 1;
    RobotLeg = Simulation7(KKnee,v,z0,RobotLeg,SoccerBall);
    
    f = -(max(RobotLeg.KinematicChains.RL.traj.y(:,10)));
    BallXSpeed(i,:) = -f;
    BallZSpeed(i,:) = max(RobotLeg.KinematicChains.RL.traj.y(:,12));
    BallSpeedMax(i,:) = max((RobotLeg.KinematicChains.RL.traj.y(:,10).^2 + ...
        RobotLeg.KinematicChains.RL.traj.y(:,11).^2 + ...
        RobotLeg.KinematicChains.RL.traj.y(:,12).^2).^(1/2));
    
    fprintf('K: %f,   Ball X Speed: %f,   Ball Z Speed: %f,   Ball Speed: %f,   Function: %f\n',...
        KKnee,-f,BallZSpeed(i,:),BallSpeedMax(i,:),f);
end
figure
plot(KKneeHist,BallXSpeed,KKneeHist,BallZSpeed(1:16),KKneeHist,BallSpeedMax(1:16));
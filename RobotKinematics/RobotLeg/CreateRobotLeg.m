function RobotLeg = CreateRobotLeg

fprintf('Robot Leg\n\n');

% Creates a struct to store all the robot kinematics
RobotKinematics = struct();

% Define the kinematics that are going to be added to the robot

% Robot Leg Kinematics
RobotKinematics.RL = RobotLegKinematics;

% Create the RoboHAZMAT Robot object
RKStruct = struct();
RKStruct.name = 'Robot Leg';
RKStruct.KC = RobotKinematics;
RobotLeg = Robot(RKStruct);
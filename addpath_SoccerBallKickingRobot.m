%% ==================Addpath Soccer Ball Kicking Robot=====================
% 2.740: Bio-Inspired Robotics
% Soccer Ball Kicking Robot
% Gerardo Bledt
% October 28, 2015
%
% Adds the necessary subdirectories to carry out the simulations.

%% Adds the directory containing all of Robot Kinematics
addpath RobotKinematics/
addpath RobotKinematics/RobotLeg

%% Adds the directory containing the Trajectory Planning and Optimization
addpath TrajectoryPlanningOptimization/
addpath TrajectoryPlanningOptimization/TrajectoryLibrary
addpath TrajectoryPlanningOptimization/Optimization

%% Adds all of the necessary utilities for the simulaitons
addpath Utils/
addpath Utils/Rotation
addpath Utils/Plotting
addpath Utils/Physics
addpath Utils/Misc

%% Adds the Serial communication scripts
addpath SerialCommunication/
addpath SerialCommunication/IMU
addpath SerialCommunication/MotorControl

%% Adds the Intuitive Control scripts
addpath IntuitiveRobotControl/
addpath IntuitiveRobotControl/Control
addpath IntuitiveRobotControl/Control/DynamixelControl
addpath IntuitiveRobotControl/Control/KeyboardControl
addpath IntuitiveRobotControl/Control/ServoControl
addpath IntuitiveRobotControl/StateEstimation

%% Adds the scripts containing the Simulations
addpath Simulation/
addpath Simulation/RobotLeg
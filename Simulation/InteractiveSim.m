function Robot = InteractiveSim(Robot, mode)
%% ========================Interactive Simulation==========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
% Runs an interactive Simulation for the robotic system chosen.


clc;
RobotFigure = gcf;
states = guidata(RobotFigure);
fprintf('====================================\n\n');
fprintf('     Interactive Simulation: %i\n\n', mode);
fprintf('====================================\n\n');
fprintf('       Running Simulation...\n');
fprintf('         {Delete to quit}\n\n');


Robot = Simulation1(Robot);
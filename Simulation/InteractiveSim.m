function Robot = InteractiveSim(Robot, mode)
%% ========================Interactive Simulation==========================
% RoboHAZMAT: Senior Design Project
% Motion Control Team
% Gerardo Bledt
% December 15, 2014
%
% Runs an interactive Simulation for the robotic system chosen.


clc;
FigureSetup(Robot)
RobotFigure = gcf;
states = guidata(RobotFigure);
fprintf('====================================\n\n');
fprintf('     Interactive Simulation: %i\n\n', mode);
fprintf('====================================\n\n');
fprintf('       Running Simulation...\n');
fprintf('         {Delete to quit}\n\n');

if (mode == 1)
    Robot = Simulation1(Robot);
elseif (mode == 2)
    Robot = Simulation2(Robot);
end
    
%% Cleaning the Matlab Command Window and Workspace Variables 

clear all;
clc;
close all;

%%  Initialise ROS and Dobot ======================================================================================

rosshutdown;
rosinit('192.168.27.1');

dobot = DobotMagician();
dobot.InitaliseRobot;

%%  Loadin Simulated Dobot (For Using the Function fkine and ikcon and jtraj) =====================================

% Create the Links and the end-effector of Dobot (Using Dobot's DH parameters)
L1 = Link('d',0,       'a',0,       'alpha',-pi/2);  % Base 
L2 = Link('d',0,       'a',0.135,   'alpha',0);      % RearArm
L3 = Link('d',0,       'a',0.147,   'alpha',0);      % ForeArm
L4 = Link('d',0,       'a',0.06,    'alpha',pi/2);   % End-Effector Bracket
L5 = Link('d',-0.06,   'a',0,       'alpha',0);      % Suction Cup

% Specify the joint limits 
L1.qlim = [-135 135]*pi/180;
L2.qlim = [5 80]*pi/180;
L3.qlim = [15 170]*pi/180;
L4.qlim = [-90 90]*pi/180;
L5.qlim = [-85 85]*pi/180;

L2.offset = -pi/2;
L3.offset =  pi/4;
L4.offset = -pi/4;

% default joint state (q)
q = [0 pi/4 pi/4 0 0];

% Use serial link to connect the links together
DobotSim.model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot');

% Calculate the Target Pose of the Coaster (Robot's Frame of Reference)
coasterHeight = -0.12;

% Define the target pose for picking up the coaster (you can set these manually or calculate them)

% Uncomment if you want to plot the stick model Dobot 
DobotSim.model.plot(q);

%% Control the Real Dobot  ======================================================================================

q0 = [0 pi/4 pi/4 0 0];            % Dobot initial joint state (startup position)
q1 = [-0.9554 0.3962 0.1853 0 0];  % A point above the coaster
q2 = [0.5849 0.1784 -0.0749 0 0];  % A point above the end point of the coaster (The tray)

% Set the initial joint state to the Dobot's start position
qI = q0;

% Move to the position above the coaster
qMatrix =  jtraj(qI, q1, 50);

% Add code here to control your Dobot to move to qMatrix

% Move down to pick up the coaster
qMatrix2 = jtraj(q1, q2, 50);

% Add code here to control your Dobot to move to qMatrix2

% Add code here to turn on the suction cup

% Move back to a safe position
qMatrix3 = jtraj(q2, q1, 50);

% Add code here to control your Dobot to move to qMatrix3

% Add code here to turn off the suction cup

% Return to the initial position
qMatrix4 = jtraj(q1, q0, 50);

% Add code here to control your Dobot to move to qMatrix4
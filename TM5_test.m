function TM5_test
close all
clear 
clc

% L1 = Link('d',1,'a',0,'alpha',-1.57,'qlim',[-pi pi]) 
% L2 = Link('d',1,'a',2,'alpha',1.57,'qlim',[-pi pi]) 
% L3 = Link('d',1,'a',0,'alpha',-1.57,'qlim',[-pi pi]) 
% L4 = Link('d',1,'a',0,'alpha',0,'qlim',[-pi pi]) 
% L5 = Link('d',1,'a',0,'alpha',0,'qlim',[-pi pi])
% L6 = Link('d',0,'a',0,'alpha',0,'qlim',[-pi pi])

%DH parameters from UR3 robot in toolbox
L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

% Generate the model
robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot')          
            
% Creates a vector of n joint angles at 0.
q = zeros(6,robot.n); 
%q = [0, -1.2, -1.57, -1.5640, 0, 0];

% Set the size of the workspace when drawing the robot
workspace = [-4 4 -4 4 -2 8];
scale = 0.5;

% Plot the robot
robot.plot(q,'workspace',workspace,'scale',scale);

% 4.3 Manually play around with the robot
robot.teach();

% 4.4 Get the current joint angles based on the position in the model
q = robot.getpos()  

% 4.5 Get the joint limits
robot.qlim 
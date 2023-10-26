% Create an instance of the DobotM class
dobot = DobotM();

% Define the initial and target end effector positions (in meters)
initialPosition = [0.2, 0.2, 0.2];  % Replace with your desired initial position
targetPosition = [0.5, 0.2, 0.7];  % Replace with your desired target position

% Use inverse kinematics to compute joint angles for the target position
targetJointAngles = dobot.model.ikine(transl(targetPosition), dobot.q, [1, 1, 1, 0, 0, 0]);

% Time duration for the motion (in seconds)
motionDuration = 2.0;  % Adjust as needed

% Number of discrete time steps for the motion
numSteps = 100;  % Adjust as needed

% Interpolate joint angles from initial to target
jointAnglesTrajectory = linspace(dobot.q, targetJointAngles, numSteps);

% Perform the motion
for i = 1:numSteps
    % Set the robot's joint angles
    dobot.q = jointAnglesTrajectory(i, :);
    
    % Plot and update the robot's position
    dobot.PlotAndColourRobot();
    
    % Pause to control the motion speed
    pause(motionDuration / numSteps);
end
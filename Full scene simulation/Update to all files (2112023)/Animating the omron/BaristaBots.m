function BaristaBots()
    close all

    % Load in the Omron TM5 robot model
    r = OmronTM5;  % Use the OmronTM5 class

    r2 = DobotMagician(transl(1.3,1.25,1.25));
    % Setting up workspace size
    axis([-4, 4, -4, 4, 0, 5])

    % cup setup
    hold on

    % Define cup positions and orientations as arrays
    cup_positions = [
        [-1, 2.1, 2];
        [-1.1, 2.1, 2];
        % Add more cup positions as needed
    ];

    cup_orientations = [
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        % Add more cup orientations as needed
    ];

    % Define the cup wall cup positions and orientations as arrays
    goal_positions = [
        [-1, 1, 2];
        [-1.1, 1, 2];
        % Add more goal positions as needed
    ];

    goal_orientations = [
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        % Add more goal orientations as needed
    ];

    % Create an empty array to hold cup objects to spawn in
    cups = [];

    % Iterate through the cup positions and create cup objects
    for i = 1:size(cup_positions, 1)
        cup = PlaceObject('cup.ply', cup_positions(i, :));
        cups = [cups, cup];
    end


    % Scale and position the robot model
    %scaleFactor = 2;
    r.model.base = transl(-1, 1.5, 2); % Example base position, adjust as needed

   % r2.model.base = transl(1.3,1.25,1.45);
   % r2.TestMoveJoints();
    %environment and safety object setup and import
    PlaceObject('CoffeeBarScaled.ply', [-0.5,1.5,1.2]);
    PlaceObject('emergencyStopWallMounted.ply', [-2.55,1.8,1.5]);
    PlaceObject('emergencyStopWallMountedtwo.ply', [0.8, 2.5,1.5]);
    PlaceObject('fireExtinguisher.ply', [1, 2.5, 1]);
    PlaceObject('fireExtinguisher.ply', [-2.55, 1.5, 1]);
    PlaceObject('couch.ply', [-3.8, -0.8, 0.15]);
    PlaceObject('coaster.ply', [1.25,1.75,1.45]);
    PlaceObject('coaster.ply', [1.25,1.75,1.46]);
    PlaceObject('coaster.ply', [1.25,1.75,1.47]);
    PlaceObject('coaster.ply', [1.25,1.75,1.48]);
    PlaceObject('art.ply', [3,4,3]);
    PlaceObject('art.ply', [2,4,2]);
    PlaceObject('art.ply', [1,4,3]);
    PlaceObject('art.ply', [0,4,4]);
    PlaceObject('art.ply', [-1,4,3]);
    PlaceObject('art.ply', [-2,4,2]);
    PlaceObject('art.ply', [-3,4,3]);
    PlaceObject('art2.ply', [-4,3,3]);
    PlaceObject('art2.ply', [-4,2,2]);
    PlaceObject('art2.ply', [-4,1,3]);
    PlaceObject('art2.ply', [-4,0,4]);
    PlaceObject('art2.ply', [-4,-1,3]);
    PlaceObject('art2.ply', [-4,-2,2]);
    PlaceObject('art2.ply', [-4,-3,3]);
    PlaceObject('VaseScaled.ply', [-2,0.6,2.4]);
    PlaceObject('VaseScaled.ply', [-1,0.6,2.4]);
    PlaceObject('VaseScaled.ply', [0,0.6,2.4]);
    PlaceObject('lamp.ply', [3.5,-3.5,0.1]);
    PlaceObject('lamp.ply', [3.5,3.5,0.1]);
    PlaceObject('pendantlight.ply', [-2, -2,3]);
    % Inclusion of the wood floor via image texture
    surf([-4, -4; 4, 4], [-4, 4; -4, 4], [0.01, 0.01; 0.01, 0.01], 'CData', imread('wood.jpg'), 'FaceColor', 'texturemap');

    % Initialize step counter
    current_step = 1;

    % Get a trajectory for robot joint angles
    qPath = jtraj(r.model.qlim(:, 1)', r.model.qlim(:, 2)', 200);

    qPath2 = jtraj(r2.model.qlim(:, 1)', r2.model.qlim(:, 2)', 200);

    q0 = r2.model.getpos();
    T1 = transl(1.25,1.75,1.28);
    q1 = r2.model.ikcon(T1);
    qTraj1 = jtraj(q0,q1,100);

    for i = 1:size(qTraj1,1)
        q = qTraj1(i,:);
        r2.model.animate(q);   
        pause(0.01)  
    end
    
    % Animate the model along the qPath created
    for i = 1:length(qPath)
        r.model.animate(qPath(i, :));
        drawnow();

        % Determine the current cup and goal positions and orientations
        current_cup_position = cup_positions(current_step, :);
        current_cup_orientation = cup_orientations(current_step, :);
        current_goal_position = goal_positions(current_step, :);
        current_goal_orientation = goal_orientations(current_step, :);

        % Move to the current cup position and orientation
        MoveToEndEffectorToCup(r, current_cup_position, current_cup_orientation);
        pause(0.5)

        % Move to the current goal position and orientation
        MoveToEndEffectorToGoal(r, current_goal_position, current_goal_orientation);
        pause(0.5)

        % Check if a cup needs to be placed at the goal position
        if current_step <= numel(cup_positions)
            % Create a new cup object at the goal position and orientation
            new_cup = PlaceObject('cup.ply', current_goal_position);
            cups = [cups, new_cup]; % Add the new cup to the array
        end

        % Increment the step counter or reset if all steps are completed
        if current_step < numel(cup_positions) + 1
            current_step = current_step + 1;
        else
            current_step = 1; % Reset to the first step
        end
    end
end

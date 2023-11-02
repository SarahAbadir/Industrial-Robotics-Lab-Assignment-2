function Linear_OmronTM5_test()
    close all

    % Load in the Omron TM5 robot model
    r = OmronTM5;  % Use the OmronTM5 class instead of LinearUR3

    % Setting up workspace size
    axis([-3, 3, -3, 3, 0, 4])

    % Brick setup
    hold on

    % Define brick positions and orientations as arrays
    brick_positions = [
        [0, 0.5, 0.2];
        [0.5, 0, 1.1];
        % Add more brick positions as needed
    ];

    brick_orientations = [
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        % Add more brick orientations as needed
    ];

    % Define the brick wall brick positions and orientations as arrays
    goal_positions = [
        [0, 0.7, 0.2];
        [0.7, 0, 1.1];
        % Add more goal positions as needed
    ];

    goal_orientations = [
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        deg2rad([0, 180, -90, 0, 0, 90, 0]);
        % Add more goal orientations as needed
    ];

    % Create an empty array to hold brick objects to spawn in
    bricks = [];

    % Iterate through the brick positions and create brick objects
    for i = 1:size(brick_positions, 1)
        brick = PlaceObject('brick.ply', brick_positions(i, :));
        bricks = [bricks, brick];
    end

    % Environment and safety object setup and import
    % Add your environment objects here

    % Scale and position the robot model
    scaleFactor = 2;
    r.model.base = transl(0, 0, 0.1); % Example base position, adjust as needed

    % Inclusion of the concrete floor via image texture
    surf([-2.8, -2.8; 2.8, 2.8], [-2.8, 2.8; -2.8, 2.8], [0.01, 0.01; 0.01, 0.01], 'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');

    % Initialize step counter
    current_step = 1;

    % Get a trajectory for robot joint angles
    qPath = jtraj(r.model.qlim(:, 1)', r.model.qlim(:, 2)', 200);

    % Animate the model along the qPath created
    for i = 1:length(qPath)
        r.model.animate(qPath(i, :));
        drawnow();

        % Determine the current brick and goal positions and orientations
        current_brick_position = brick_positions(current_step, :);
        current_brick_orientation = brick_orientations(current_step, :);
        current_goal_position = goal_positions(current_step, :);
        current_goal_orientation = goal_orientations(current_step, :);

        % Move to the current brick position and orientation
        MoveToEndEffectorToBrick(r, current_brick_position, current_brick_orientation);
        pause(0.5)

        % Move to the current goal position and orientation
        MoveToEndEffectorToGoal(r, current_goal_position, current_goal_orientation);
        pause(0.5)

        % Check if a brick needs to be placed at the goal position
        if current_step <= numel(brick_positions)
            % Create a new brick object at the goal position and orientation
            new_brick = PlaceObject('brick.ply', current_goal_position);
            bricks = [bricks, new_brick]; % Add the new brick to the array
        end

        % Increment the step counter or reset if all steps are completed
        if current_step < numel(brick_positions) + 1
            current_step = current_step + 1;
        else
            current_step = 1; % Reset to the first step
        end
    end
end

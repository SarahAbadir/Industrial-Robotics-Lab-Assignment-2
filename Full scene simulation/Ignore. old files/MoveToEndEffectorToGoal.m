function MoveToEndEffectorToGoal(robot, goal_positions, goal_orientations)
    % Get the current joint angles
    q = robot.model.getpos();

    % Define the number of steps for the trajectory
    steps = 200;

    % Iterate through the target positions and orientations
    for i = 1:size(goal_positions, 1)
        % Define the desired end effector position and orientation
        goal_position = goal_positions(i, :);
        goal_orientation = goal_orientations(i, :);

        % Calculate the transformation matrix (T) for the target pose
        T = transl(goal_position) * rpy2tr(goal_orientation(1), goal_orientation(2), goal_orientation(3));

        % Calculate the joint angles (q_target) to reach the desired pose
        q_target = wrapToPi(robot.model.ikcon(T, q));

        % Generate a joint trajectory from the current configuration (q) to q_target
        qTraj = jtraj(q, q_target, steps);

        % Iterate through the trajectory and animate the robot
        for j = 1:size(qTraj, 1)
            q = qTraj(j, :);
            robot.model.animate(q);
            pause(0.01);
        end

        % Update the current joint angles
        q = q_target;
    end
end
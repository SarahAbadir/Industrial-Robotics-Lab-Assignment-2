function MoveToEndEffectorToGoal(robot, goal_positions, goal_orientations)
    % Get the current joint angles
    q = robot.model.getpos();
    
    % Define the number of steps for the trajectory
    steps = 200;

    % Iterate through the goal positions and orientations
    for i = 1:size(goal_positions, 1)
        % Define the desired end effector position and orientation
        goal_position = goal_positions(i, :);
        goal_orientation = goal_orientations(i, :);
        
        % Calculate the transformation matrix (T) for the goal pose
        T = transl(goal_position) * trotz(pi) * trotz(pi) * trotz(pi) * trotz(pi) * trotz(pi);
        
        % Calculate the joint angles (q_goal) to reach the desired pose using ikine
        q_goal = robot.model.ikine(T, 'mask', [1 1 1 1 1 1], 'q0', q);
        
        % Apply joint limits
        q_goal = min(max(q_goal, robot.model.qlim(:, 1)'), robot.model.qlim(:, 2)');
        
        % Generate a joint trajectory from the current configuration (q) to q_goal
        qTraj = jtraj(q, q_goal, steps);
        
        % Iterate through the trajectory and animate the robot
        for j = 1:size(qTraj, 1)
            q = qTraj(j, :);
            robot.model.animate(q);
            drawnow();
        end
        
        % Update the current joint angles
        q = q_goal;
    end
end

function MoveToEndEffectorToCup(robot, target_positions, target_orientations)
    % Get the current joint angles
    q = robot.model.getpos();
    
    % Define the number of steps for the trajectory
    steps = 200;

    % Iterate through the target positions and orientations
    for i = 1:size(target_positions, 1)
        % Define the desired end effector position and orientation
        target_position = target_positions(i, :);
        target_orientation = target_orientations(i, :);
        
        % Calculate the transformation matrix (T) for the target pose
        T = transl(target_position) * trotz(pi) * trotz(pi) * trotz(pi) * trotz(pi) * trotz(pi);
        
        % Calculate the joint angles (q_target) to reach the desired pose using ikine
        q_target = robot.model.ikine(T, 'mask', [1 1 1 1 1 1], 'q0', q);
        
        % Apply joint limits
        q_target = min(max(q_target, robot.model.qlim(:, 1)'), robot.model.qlim(:, 2)');
        
        % Generate a joint trajectory from the current configuration (q) to q_target
        qTraj = jtraj(q, q_target, steps);
        
        % Iterate through the trajectory and animate the robot
        for j = 1:size(qTraj, 1)
            q = qTraj(j, :);
            robot.model.animate(q);
            drawnow();
        end
        
        % Update the current joint angles
        q = q_target;
    end
end

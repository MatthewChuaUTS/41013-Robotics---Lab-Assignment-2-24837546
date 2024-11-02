function [robotTrajectoryQmatrix] = calculateRobotTrajectory(robot, trMatrix, steps)

    numSteps = size(trMatrix, 2); % gets the amount of joints needed to calculate
    robotTrajectoryQmatrix = cell(1, numSteps - 1);
    qWaypoints = zeros(numSteps, 6);
    robotCurrentJointPosition = robot.model.getpos();
    
    % Generate the joint angle using ikcon
    for i = 1:numSteps
        qWaypoints(i, :) = robot.model.ikcon(trMatrix{i}, robotCurrentJointPosition);
        robotCurrentJointPosition = qWaypoints(i, :); 
    end
    for i = 1:(numSteps - 1)
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps); % Quintic Polynomial Interpolation
        robotTrajectoryQmatrix{i} = traj;
    end
end



function [robotTrajectoryQmatrix] = calculateRobotTrajectory(robot, trMatrix, steps)

    numSteps = size(trMatrix, 2);
    robotTrajectoryQmatrix = cell(1, numSteps - 1);
    qWaypoints = zeros(numSteps, 6);
    robotCurrentJointPosition = robot.model.getpos();
    
    % Generate the joint angle using ikcon
    for i = 1:numSteps
        qWaypoints(i, :) = robot.model.ikcon(trMatrix{i}, robotCurrentJointPosition);
        robotCurrentJointPosition = qWaypoints(i, :); 
    end

    for i = 1:(numSteps - 1)
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
        robotTrajectoryQmatrix{i} = traj;
    end
end



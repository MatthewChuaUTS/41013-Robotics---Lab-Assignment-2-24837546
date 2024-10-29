function [robotTrajectoryQmatrix, collisionFlag] = calculateRobotTrajectory(robot, trMatrix, steps, faces, vertex, faceNormals)

    collisionFlag = 0;
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
        if IsCollision(robot, traj, faces, vertex, faceNormals)
            collisionFlag = 1;
            disp('collision found during trajectory calculation at waypoint:');
            disp(i);
            break;
        end
        robotTrajectoryQmatrix{i} = traj;
    end
end



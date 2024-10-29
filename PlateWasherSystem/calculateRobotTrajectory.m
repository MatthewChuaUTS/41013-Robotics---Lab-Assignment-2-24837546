% function [robotTrajectoryQmatrix] = calculateRobotTrajectory(robot, trMatrix, steps)
% 
%     numSteps = size(trMatrix, 2);
%     qmatrix = zeros((numSteps - 1) * steps, 6);
%     qWaypoints = zeros(numSteps, 6);
%     robotCurrentJointPosition = robot.model.getpos();
% 
%     % Generate the joint angle using ikcon
%     for i = 1:numSteps
%         qWaypoints(i, :) = robot.model.ikcon(trMatrix{i}, robotCurrentJointPosition);
%         robotCurrentJointPosition = qWaypoints(i, :); 
%     end
% 
%     rowIdx = 1;  
%     for i = 1:(numSteps - 1) 
%         traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
%         qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;  
%         rowIdx = rowIdx + steps;
%     end
%     robotTrajectoryQmatrix = qmatrix;
% end
function [robotTrajectoryQmatrix, collisionFlag] = calculateRobotTrajectory(robot, trMatrix, steps)

    collisionFlag = 0; % 1 when collision is present, 0 when no collision
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



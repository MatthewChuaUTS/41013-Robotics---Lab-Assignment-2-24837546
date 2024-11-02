function [niryoTrajectoryQmatrix] = calculateNiryoTrajectory(myNiryoOne, steps)
    trSteps = {transl([0.22,0.061,0.195]) ...
        , transl([0.327,0.232,0.244]) ...
        , transl([0.358,0.196,0.005]) ...
        , transl([0.381,0.171,0.406]) ...
        , transl([0.678,0.187,0.406]) ...
        , transl([0.678,0.187,0.406]) ...
        , transl([0.678,0.187,0.406]) ...
        , transl([0.747,-0.049,0.072]) ... 
        , transl([0.655,0.074,0.457]) ...
        , transl([0.655,0.074,0.457]) ...
        , transl([0.747,-0.049,0.072]) ...
        , transl([0.22,0.061,0.195])};

    % Preallocate qmatrix for 550 rows (11 segments x 50 steps) and 6 columns
    % (more rows for this one, will eventually made it equal to ur3)
    steps = 50;
    qmatrix = zeros((length(trSteps) - 1) * steps, 6);
    qWaypoints = zeros((length(trSteps)) * steps, 6); 
    
    % Generate the joint angle using ikcon
    for i = 1:length(trSteps)
        qWaypoints(i, :) = myNiryoOne.model.ikcon(trSteps{i}, niryoOneCurrentJointPosition);
        niryoOneCurrentJointPosition = qWaypoints(i, :); 
    end
    
    % Generate the trajectory and store it in the preallocated matrix
    rowIdx = 1;  
    for i = 1:(length(trSteps) - 1) 
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
        if i >= 9 && i <= 10 
            for j = 1:steps
                traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps) * roty(pi/2);  
            end
        end
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;
        rowIdx = rowIdx + steps;
        disp(rowIdx);
    end
    
    niryoTrajectoryQmatrix = qmatrix;

    % Animate the NiryoOne robot along the generated trajectory
    for j = 1:size(qmatrix, 1)
        myNiryoOne.model.animate(qmatrix(j, :));
        drawnow();  
        pause(0.01); 
    end
end

env = environment();  %%
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));
niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];  
myNiryoOne.model.animate(niryoOneCurrentJointPosition); 


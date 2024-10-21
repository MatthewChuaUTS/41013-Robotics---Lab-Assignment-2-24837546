% function [niryoTrajectoryQmatrix] = calculateNiryoTrajectory(myNiryoOne, totalSteps)
% end

env = environment();  %%
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));

% Define the initial joint configuration
niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];  
myNiryoOne.model.animate(niryoOneCurrentJointPosition); % Move to initial position

% Define trsteps for the UR3 robot
trSteps = {transl([0.22,0.061,0.195]) ...
        , transl([0.327,0.232,0.244]) ...
        , transl([0.358,0.196,0.005]) ...
        , transl([0.381,0.171,0.406]) ...
        , transl([0.678,0.187,0.406]) ...
        , transl([0.678,0.187,0.406]) ...
        , transl([0.678,0.187,0.406]) ...
        , transl([0.747,-0.049,0.072]) ... % * trotx(pi/180)
        , transl([0.655,0.074,0.457]) ...
        , transl([0.655,0.074,0.457]) ...
        , transl([0.655,0.074,0.457]) ...
        , transl([0.655,0.074,0.457]) ...
        , transl([0.655,0.074,0.457]) ...
        , transl([0.747,-0.049,0.072]) ...
        , transl([0.22,0.061,0.195])};

% Preallocate qmatrix for 550 rows (11 segments x 50 steps) and 6 columns
% (more rows for this one, will eventually made it equal to ur3)
% q2 = UR3.model.ikcon(b1_upos, q1);
% Generate the trajectory and store it in the preallocated matrix
steps = 50;
qmatrix = zeros((length(trSteps) - 1) * steps, 6);

% Generate the trajectory and store it in the preallocated matrix
rowIdx = 1;  % Row index for storing trajectories
for i = 2:length(trSteps)
    % Generate trajectory for the current segment
    traj = myNiryoOne.model.ikcon(trSteps(i, :), niryoOneCurrentJointPosition);
    
    % Store the trajectory in the preallocated qmatrix
    qmatrix(rowIdx:rowIdx + steps - 1, :) = traj; % qmatrix adds another 50 at the array every i
    
    % Update the row index for the next segment before exit the loop
    niryoOneCurrentJointPosition = niryoOneCurrentJointPosition; 

    % Update the row index for the next segment
    rowIdx = rowIdx + steps;
    disp(rowIdx);
end

% Animate the NiryoOne robot along the generated trajectory
for j = 1:size(qmatrix, 1)
    myNiryoOne.model.animate(qmatrix(j, :));
    drawnow();  % Ensure real-time rendering
    pause(0.01);  % Add a small pause for smooth animation
end
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
        , transl([0.747,-0.049,0.072]) ...
        , transl([0.22,0.061,0.195])};

% Preallocate qmatrix for 550 rows (11 segments x 50 steps) and 6 columns
% (more rows for this one, will eventually made it equal to ur3)
steps = 50;
qmatrix = zeros((length(trSteps) - 1) * steps, 6); % array for traj
qWaypoints = zeros((length(trSteps)) * steps, 6); % array for ikcon

% Generate the joint angle using ikcon
for i = 1:length(trSteps)
    % Generate trajectory for the current segment
    qWaypoints(i, :) = myNiryoOne.model.ikcon(trSteps{i}, niryoOneCurrentJointPosition);
   
    % Update the CurrentJointPosition for the next segment before exit the loop
    niryoOneCurrentJointPosition = qWaypoints(i, :); 
end

% Generate the trajectory and store it in the preallocated matrix
rowIdx = 1;  % Row index for storing trajectories
amplitude = deg2rad(5);  % Amplitude of the circular motion (5 degrees)
frequency = 2 * pi / steps;  % Frequency of the circular motion 

for i = 1:(length(trSteps) - 1) % stop at 2nd last
    % Generate trajectory for the current segment
    traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
    
    % Only apply circular motion between specific waypoints
    if i >= 9 && i <= 10  % These are the waypoints where circular motion occurs
        for j = 1:steps
            % Apply circular motion to joints 9-11
            traj(j, 3) = traj(j, 3) + amplitude * sin(frequency * j);  % Sine wave for joint 3
            traj(j, 4) = traj(j, 4) + amplitude * cos(frequency * j);  % Cosine wave for joint 4
            traj(j, 5) = traj(j, 5) + amplitude * sin(frequency * j);  % Sine wave for joint 5
        end
    end

    % Store the trajectory in the preallocated qmatrix
    qmatrix(rowIdx:rowIdx + steps - 1, :) = traj; % qmatrix adds another 50 at the array every i
    
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
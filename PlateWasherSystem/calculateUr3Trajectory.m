% % function [ur3TrajectoryQmatrix] = calculateUr3Trajectory(myUR3, totalSteps)
% % end

clf;
clear all; %#ok<CLALL>

% Initialize the environment and UR3 robot model
env = environment();  %%
myUR3 = UR3(transl(1.02, -0.01, 0));

% Define the initial joint configuration
UR3InitialJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];
myUR3.model.animate(UR3InitialJointPosition);  % Move to initial position

% Define waypoints for the UR3 robot
qWaypoints = [UR3InitialJointPosition;
    deg2rad([0, -57.1, 131, -163, -90, 0]);
    deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
    deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
    deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
    deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
    deg2rad([0, -57.1, 131, -163, -90, 0]);
    deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
    deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
    deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
    deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
    deg2rad([-134, -74.3, 111, -12.9, -90, -32.1])];

% Preallocate qmatrix for 550 rows (11 segments x 50 steps) and 6 columns
steps = 50;
qmatrix = zeros((length(qWaypoints) - 1) * steps, 6);

% Generate the trajectory and store it in the preallocated matrix
rowIdx = 1;  % Row index for storing trajectories
amplitude = deg2rad(5);  % Amplitude of the circular motion (5 degrees)
frequency = 2 * pi / steps;  % Frequency of the circular motion 

for i = 2:length(qWaypoints)
    % Generate trajectory for the current segment
    traj = jtraj(qWaypoints(i - 1, :), qWaypoints(i, :), steps);
    
    % Only apply circular motion between specific waypoints
    if i >= 3 && i <= 5  % These are the waypoints where circular motion occurs
        for j = 1:steps
            % Apply circular motion to joints 3-5
            traj(j, 3) = traj(j, 3) + amplitude * sin(frequency * j);  % Sine wave for joint 3
            traj(j, 4) = traj(j, 4) + amplitude * cos(frequency * j);  % Cosine wave for joint 4
            traj(j, 5) = traj(j, 5) + amplitude * sin(frequency * j);  % Sine wave for joint 5
        end
    end

    % Store the trajectory in the preallocated qmatrix
    qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;
    
    % Update the row index for the next segment
    rowIdx = rowIdx + steps;
    disp(rowIdx);
end

% Animate the UR3 robot along the generated trajectory
for j = 1:size(qmatrix, 1)
    myUR3.model.animate(qmatrix(j, :));
    drawnow();  % Ensure real-time rendering
    pause(0.01);  % Add a small pause for smooth animation
end
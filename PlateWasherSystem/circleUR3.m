function circleUR3(myUR3)
    % Set the limits and step size
    limit = 2 * pi;   % 360 degrees in radians
    step = limit / 50; % Step size

    % Initialize angle and direction
    current_angle = 0;
    direction = 1; % 1 for forward, -1 for backward

    % Infinite loop to move the robot back and forth
    while true
        % Update the current angle based on direction
        current_angle = current_angle + direction * step;

        % Animate the robot with the updated angle on the first joint
        myUR3.model.animate([current_angle, 0, 0, 0, 0, 0]);

        % Check if the angle exceeds the limits (360 degrees)
        if current_angle >= limit || current_angle <= 0
            % Reverse direction when limit is reached
            direction = -direction;
        end

        % Add a small pause to visualize the movement
        pause(0.01);
    end
end

%% dump

% Correct the input points for the trajectory (circular motion or specific points)
% Define points in a circle or linear path for example
theta = linspace(0, 2*pi, 100);  % For a circular motion
radius = 0.05;  % Radius of circular motion
X = radius * cos(theta) + 0.75;  % Center of the circle at x = 0.75
Y = radius * sin(theta) + 0.008; % Center of the circle at y = 0.008
Z = repmat(0.072, size(X));      % Z stays constant at 0.072

% Step 2: Transform the points using the end effector transformation matrix
endEffectorTr = myUR3.model.fkine(UR3CurrentJointPosition);  % No .T needed, it returns the 4x4 matrix
updatedPoints = endEffectorTr * [X(:), Y(:), Z(:), ones(numel(X), 1)]';  % Apply transformation

% Step 3: Solve the inverse kinematics using 'ikine'
% 'ikine' expects a homogeneous transformation matrix, so make one from each point
for i = 1:numel(X)
    targetPose = transl(updatedPoints(1,i), updatedPoints(2,i), updatedPoints(3,i));
    circularMotion(:, i) = myUR3.model.ikine(targetPose, 'q0', UR3CurrentJointPosition, 'mask', [1,1,1,0,0,0]);
end

% Plot or animate the robot's motion (optional)
for i = 1:size(circularMotion, 2)
    myUR3.model.animate(circularMotion(:, i));
    drawnow;
end
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

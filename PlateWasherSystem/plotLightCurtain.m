function plotLightCurtain()
    % plotLightCurtain - This function plots a vertical array of laser beams
    % representing a light curtain in 3D space. The light curtain spans from
    % a bottom-left point to a top-right point with evenly spaced lasers.
    % This is only a visual representation of the intersection within
    % RobotPiggy. Check that class to see how the intersection actually
    % works. THIS IS ONLY FOR LOOKS
    
    % Define the coordinates of the bottom-left corner of the light curtain
    bottomLeft = [0, -1, -0.7971];
    
    % Define the coordinates of the top-right corner of the light curtain
    topRight = [1.45, -1, 0];
    
    % Define the spacing between each laser along the Z-axis
    laserCenters = 0.1; % units in meters
    
    % Calculate the number of lasers based on the spacing
    laserArraySize = 1 / laserCenters;
    
    % Initialize matrices to store start and end points of each laser
    laserStartPoint = zeros(laserArraySize, 3);
    laserEndPoint = zeros(laserArraySize, 3);
    
    % Loop to calculate the start and end points for each laser beam
    for i = 1:laserArraySize
        % Compute the Z-coordinate for the current laser based on spacing
        z = bottomLeft(3) + (i - 1) * laserCenters;
        
        % Set the start and end points for the current laser beam
        laserStartPoint(i, :) = [bottomLeft(1), bottomLeft(2), z];
        laserEndPoint(i, :) = [topRight(1), topRight(2), z];
    end
    
    % Loop to plot each laser beam
    for i = 1:laserArraySize
        % Hold on to ensure all points and lines are plotted on the same figure
        hold on;
        
        % Plot the start and end points of the current laser beam
        plot3(laserStartPoint(i, 1), laserStartPoint(i, 2), laserStartPoint(i, 3), 'r*');
        plot3(laserEndPoint(i, 1), laserEndPoint(i, 2), laserEndPoint(i, 3), 'r*');
        
        % Plot the line connecting the start and end points of the laser beam
        plot3([laserStartPoint(i, 1), laserEndPoint(i, 1)], ...
              [laserStartPoint(i, 2), laserEndPoint(i, 2)], ...
              [laserStartPoint(i, 3), laserEndPoint(i, 3)], 'r');
        
        % Set the axis scales to be equal for better visualization
        axis equal;
    end
end

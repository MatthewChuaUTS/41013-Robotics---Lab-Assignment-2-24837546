function plotLightCurtain()
    bottomLeft = [0, -1, -0.7971];
    topRight = [1.45, -1, 0];
    laserCenters = 0.1;
    laserArraySize = 1/laserCenters;
    laserStartPoint = zeros(laserArraySize, 3);
    laserEndPoint = zeros(laserArraySize, 3);
    
    for i = 1:laserArraySize
        z = bottomLeft(3) + (i-1) * laserCenters;
        laserStartPoint(i, :) = [bottomLeft(1), bottomLeft(2), z];
        laserEndPoint(i, :) = [topRight(1), topRight(2), z];
    end

    for i = 1 : laserArraySize           
        hold on;
        plot3(laserStartPoint(i, 1),laserStartPoint(i, 2),laserStartPoint(i, 3) ,'r*');
        plot3(laserEndPoint(i, 1),laserEndPoint(i, 2),laserEndPoint(i, 3) ,'r*');
        plot3([laserStartPoint(i, 1),laserEndPoint(i, 1)],[laserStartPoint(i, 2),laserEndPoint(i, 2)],[laserStartPoint(i, 3),laserEndPoint(i, 3)] ,'r');
        axis equal
    
    end
end
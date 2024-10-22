function [ur3TrajectoryQmatrix] = calculateUr3Trajectory(myUR3, steps)
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
        deg2rad([-134, -78.3, 102, 250, -93.9, 110])];
    
    steps = 50;
    qmatrix = zeros((length(qWaypoints) - 1) * steps, 6);
    
    rowIdx = 1;  
    for i = 2:length(qWaypoints)
        traj = jtraj(qWaypoints(i - 1, :), qWaypoints(i, :), steps);
        if i >= 2 && i <= 5  
            for j = 1:steps
                traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps);  
            end
        end
       
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;    
        rowIdx = rowIdx + steps;
        disp(rowIdx);
    end

    ur3TrajectoryQmatrix = qmatrix;
    
    for j = 1:size(qmatrix, 1)
        myUR3.model.animate(qmatrix(j, :));
        drawnow(); 
        pause(0.01);  
    end

end

env = environment();  %%
myUR3 = UR3(transl(1.02, -0.01, 0));
UR3InitialJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];
myUR3.model.animate(UR3InitialJointPosition); 
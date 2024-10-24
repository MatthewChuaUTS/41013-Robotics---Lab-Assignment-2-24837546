% function [ur3TrajectoryQmatrix] = calculateUr3Trajectory(myUR3, steps)
env = environment();  %%
myUR3 = UR3(transl(1.02, -0.01, 0));
UR3InitialJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];
myUR3.model.animate(UR3InitialJointPosition); 

    qWaypoints = [[0, -pi/2, 0, 0, -pi/2, 0];
        deg2rad([0, -57.1, 131, -163, -90, 0]);
        deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
        deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
        deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
        deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
        deg2rad([0, -57.1, 131, -163, -90, 0]);
        deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
        deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
        deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
        deg2rad([-71.4, -11.4, 65.7, -146, -90, -32.1]);
        deg2rad([-134, -78.3, 102, 250, -93.9, 110])];
    
    steps = 50;
    qmatrix = zeros((length(qWaypoints) - 1) * steps, 6);
    deltaT = 0.05; 
    
    rowIdx = 1;  
    for i = 2:length(qWaypoints)
        traj = jtraj(qWaypoints(i - 1, :), qWaypoints(i, :), steps);
        % qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;

        if i >= 4 && i <= 6  
            for j = 1:steps
                traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps);  
            end
        end
       
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;  

        if i < length(qWaypoints)
            if i >= 11 && i <= 12
                x = zeros(2,steps);
                s = lspb(0,1,steps);                                 % Create interpolation scalar

                % calculate cells for x
                for k = 1:steps-1
                    currentTr = myUR3.model.fkine(qWaypoints(i, :)).T;
                    nextTr = myUR3.model.fkine(qWaypoints(i + 1, :)).T;
                    x(:,k) = currentTr(1:2,4) * (1-s(k)) + nextTr(1:2,4) * s(k);                  % Create trajectory in x-y plane
                end
                x(:,steps) = nextTr(1:2,4);

                % RMRC trajectory
                for j = 1:steps-1
                    xdot = (x(:,j+1) - x(:,j))/deltaT;                             % Calculate velocity at discrete time step
                    J = myUR3.model.jacob0(qmatrix(rowIdx + j - 1,:));            % Get the Jacobian at the current state
                    J = J(1:2,:);                           % Take only first 2 rows
                    qdot = pinv(J)*xdot;                             % Solve velocitities via RMRC
                    qmatrix(rowIdx + j,:) =  qmatrix(rowIdx + j - 1,:) + deltaT * qdot';                   % Update next joint state
                end
            end
        end
        rowIdx = rowIdx + steps;
        disp(rowIdx);
    end

    ur3TrajectoryQmatrix = qmatrix;
    
    for j = 1:size(qmatrix, 1)
        myUR3.model.animate(qmatrix(j, :));
        drawnow(); 
        pause(0.01);  
    end

% end


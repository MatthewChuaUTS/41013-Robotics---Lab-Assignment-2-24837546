function [ur3TrajectoryQmatrix] = calculateUr3Trajectory(myUR3, steps)
    if nargin < 2
        steps = 50;
    end
    trSteps = {transl([0.747,-0.049,0.072]) * troty(pi) ... % sponge 1
        , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 2
        , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 3
        , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 4
        , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub position 5
        , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub 6
        , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub 7
        , transl([0.747,-0.049,0.072]) * troty(pi) ... % sponge 8
        , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 9
        , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 10
        , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 11
        , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 12
        , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 13
        , transl([0.6979, 0.2587, -0.04671]) * troty(pi) ...   % dunk 14
        , transl([0.6979, 0.2587, 0.5401]) * troty(pi) ...   % up 15
        , transl([1.165, 0.3, 0.178]) * troty(pi) ... % place 16
        , transl([0.9346, -0.1223, 0.5269]) * troty(pi) ...  % reset 17
        , transl([0.9346, -0.1223, 0.5269]) * troty(pi)};  % reset 18

    qmatrix = zeros((length(trSteps) - 1) * steps, 6);
    qWaypoints = zeros((length(trSteps)) * steps, 6);  
    deltaT = 0.05; 
    UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];
    % myUR3.model.plot(UR3CurrentJointPosition); % for function test
    
    % Generate the joint angle using ikcon
    for i = 1:length(trSteps)
        qWaypoints(i, :) = myUR3.model.ikcon(trSteps{i}, UR3CurrentJointPosition);
        UR3CurrentJointPosition = qWaypoints(i, :); 
    end

    rowIdx = 1;  
    for i = 1:(length(trSteps) - 1) 
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
        if (i >= 1 && i <= 4) || (i > 6)
            x = zeros(2,steps);
            s = lspb(0,1,steps);                                 % Create interpolation scalar

            % calculate cells for x
            for k = 1:steps-1
                x(:,k) = trSteps{i}(1:2,4) * (1-s(k)) + trSteps{i+1}(1:2,4) * s(k);                  % Create trajectory in x-y plane
            end
            x(:,steps) = trSteps{i+1}(1:2,4);

            % RMRC trajectory
            for j = 1:steps-1
                xdot = (x(:,j+1) - x(:,j))/deltaT;                             % Calculate velocity at discrete time step
                J = myUR3.model.jacob0(qmatrix(rowIdx + j - 1, :));            % Get the Jacobian at the current state
                J = J(1:2,:);                           % Take only first 2 rows
                qdot = pinv(J)*xdot;                             % Solve velocitities via RMRC
                qmatrix(rowIdx + j,:) =  qmatrix(rowIdx + j - 1,:) + deltaT * qdot';                   % Update next joint state
            end
        end
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;

        if i >= 5 && i <= 6  
            for j = 1:steps
                traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps);  
            end
        end     
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;  
        rowIdx = rowIdx + steps;
        disp(rowIdx);
    end

    ur3TrajectoryQmatrix = qmatrix;

    % myUR3.model.delay = 0;
    % for j = 1:size(qmatrix, 1)
    %     myUR3.model.animate(qmatrix(j, :));
    %     drawnow(); 
    %     pause(0.01);  
    % end

end

% Call function to test it's running
% env = environment();  %%
% steps = 50;
% ur3TrajectoryQmatrix = calculateUr3Trajectory(myUR3, steps); 
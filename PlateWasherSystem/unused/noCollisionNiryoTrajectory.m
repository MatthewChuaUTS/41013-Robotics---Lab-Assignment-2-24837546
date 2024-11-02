function [niryoTrajectoryQmatrix] = noCollisionNiryoTrajectory(robo, steps) %returnOnceFound
    if nargin < 2
        steps = 50;
    end

    trSteps = [transl([0.22,0.061,0.195]) ... % plate 1
        , transl([0.358,0.196,0.244]) ... % point2 2
        , transl([0.358,0.196,-0.005]) ... % dunk 3
        , transl([0.358,0.196,0.406]) ... % point4 4  RMRC in lab happens for same axis, goes straight, is it kay if the points are short?
        , transl([0.6975,0.263,0.3915]) ... % hold 5
        , transl([0.6975,0.263,0.3915]) ... % hold 6
        , transl([0.6975,0.263,0.3915]) ... % hold 7
        , transl([0.6975,0.263,0.3915]) ... % hold 8
        , transl([0.6975,0.263,0.3915]) ... % hold 9
        , transl([0.747,-0.049,0.072]) ... % sponge 10
        , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % scrub position 11
        , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % scrub 12
        , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % srub 13
        , transl([0.726,0.1495,0.4695]) ... % scrub position 14
        , transl([0.747,-0.049,0.072]) ... % sponge 15
        , transl([0.3203,-0.01,0.423]) * trotx(-pi/2) ... % reset 16
        , transl([0.3203,-0.01,0.423]) * trotx(-pi/2) ... % reset 17
        , transl([0.3203,-0.01,0.423]) * trotx(-pi/2)]; % reset 18


    qmatrix = zeros((length(trSteps) - 1) * steps, 6);
    trSteps = zeros((length(trSteps)) * steps, 6);  
    deltaT = 0.05; 
    UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];

    
    % Generate the joint angle using ikcon
    for i = 1:length(trSteps)
        trSteps(i, :) = myUR3.model.ikcon(trSteps{i}, UR3CurrentJointPosition);
        UR3CurrentJointPosition = trSteps(i, :); 
    end

    rowIdx = 1;  
    for i = 1:(length(trSteps) - 1) 
        traj = jtraj(trSteps(i, :), trSteps(i + 1, :), steps);
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

    niryoTrajectoryQmatrix = qmatrix;
end

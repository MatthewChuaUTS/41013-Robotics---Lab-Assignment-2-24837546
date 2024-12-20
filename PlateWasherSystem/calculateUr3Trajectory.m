% function [ur3TrajectoryQmatrix] = calculateUr3Trajectory(myUR3, steps)
%     if nargin < 2
%         steps = 50;
%     end
%     trSteps = {transl([0.747,-0.049,0.072]) * troty(pi) ... % sponge 1
%         , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 2
%         , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 3
%         , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 4
%         , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub position 5
%         , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub 6
%         , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub 7
%         , transl([0.747,-0.049,0.072]) * troty(pi) ... % sponge 8
%         , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 9
%         , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 10
%         , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 11
%         , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 12
%         , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 13
%         , transl([0.6979, 0.2587, -0.04671]) * troty(pi) ...   % dunk 14
%         , transl([0.6979, 0.2587, 0.5401]) * troty(pi) ...   % up 15
%         , transl([1.165, 0.3, 0.178]) * troty(pi) ... % place 16
%         , transl([0.9346, -0.1223, 0.5269]) * troty(pi) ...  % reset 17
%         , transl([0.9346, -0.1223, 0.5269]) * troty(pi)};  % reset 18
% 
%     qmatrix = zeros((length(trSteps) - 1) * 50, 6);
%     qWaypoints = zeros((length(trSteps)) * 50, 6);  
%     deltaT = 0.05; 
%     UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];
%     % myUR3.model.plot(UR3CurrentJointPosition); % for function test
% 
%     % Generate the joint angle using ikcon
%     for i = 1:length(trSteps)
%         qWaypoints(i,:) = myUR3.model.ikcon(trSteps{i}, UR3CurrentJointPosition);
%         if any(isnan(qWaypoints(i,:)))  % If NaN is found
%             disp(['Invalid joint position at waypoint ' num2str(i)]);
%             qWaypoints(i,:) = UR3CurrentJointPosition; % Fallback to the previous position
%         end
% 
%         UR3CurrentJointPosition = qWaypoints(i,:); 
%     end
% 
%     rowIdx = 1;  
%     for i = 1:(length(trSteps) - 1) 
%         traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
%         if (i >= 13 && i <= 15) 
%             x = zeros(2,steps);
%             s = lspb(0,1,steps);                                 % Create interpolation scalar
% 
%             % calculate cells for x
%             for k = 1:steps-1
%                 x(:,k) = trSteps{i}(1:2,4) * (1-s(k)) + trSteps{i+1}(1:2,4) * s(k);                  % Create trajectory in x-y plane
%             end
%             x(:,steps) = trSteps{i+1}(1:2,4);
% 
%             % RMRC trajectory
%             for j = 1:steps-1
%                 disp(['rowIdx: ' num2str(rowIdx) ', j: ' num2str(j)]);  % Debugging print
% 
%                 xdot = (x(:,j+1) - x(:,j))/deltaT;                             % Calculate velocity at discrete time step
%                 J = myUR3.model.jacob0(qmatrix(rowIdx + j - 1, :));            % Get the Jacobian at the current state
%                 J = J(1:2,:);                           % Take only first 2 rows
%                 qdot = pinv(J)*xdot;                             % Solve velocitities via RMRC
% 
%                 if (rowIdx + j - 1) <= size(qmatrix, 1)
%                     qmatrix(rowIdx + j,:) = qmatrix(rowIdx + j - 1,:) + deltaT * qdot';
%                 else
%                     disp('Invalid index');
%                 end
%                 % qmatrix(rowIdx + j,:) =  qmatrix(rowIdx + j - 1,:) + deltaT * qdot';                   % Update next joint state
%             end
%         end
%         qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;
% 
%         if i >= 5 && i <= 6  
%             for j = 1:steps
%                 traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps);  
%             end
%         end     
%         qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;  
%         rowIdx = rowIdx + steps;
%         disp(['Updated rowIdx: ' num2str(rowIdx)]);
%     end
% 
%     ur3TrajectoryQmatrix = qmatrix;
% 
%     % myUR3.model.delay = 0;
%     % for j = 1:size(qmatrix, 1)
%     %     myUR3.model.animate(qmatrix(j, :));
%     %     drawnow(); 
%     %     pause(0.01);  
%     % end
% 
% end

% Call function to test it's running
% myUR3 = myUR3(transl(0.54, -0.01, 0));
% steps = 50;
% ur3TrajectoryQmatrix = calculateUr3Trajectory(myUR3,steps);
% myUR3.model.delay = 0;
% for j = 1:size(ur3TrajectoryQmatrix, 1)
%     myUR3.model.animate(ur3TrajectoryQmatrix(j, :));
%     drawnow(); 
%     pause(0.01);  
% end


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
    
    % myUR3 = UR3();
    UR3InitialJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];
    % myUR3.model.plot(UR3InitialJointPosition); 
    
    for i = 1:length(trSteps)
        qWaypoints(i, :) = myUR3.model.ikcon(trSteps{i}, UR3InitialJointPosition);
        UR3InitialJointPosition = qWaypoints(i, :); 
    end
    
    % Generate the trajectory and store it in the preallocated matrix
    rowIdx = 1;  
    for i = 1:(length(trSteps) - 1) 
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
        % qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;

        if i >= 5 && i <= 6  
            for j = 1:steps
                traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps);  
            end
        end
       
        % qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;  

    % if i < length(qWaypoints)
        if i >= 13 && i <= 15
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
                J = myUR3.model.jacob0(qmatrix(rowIdx + j - 1,:));            % Get the Jacobian at the current state
                J = J(1:2,:);                           % Take only first 2 rows
                qdot = pinv(J)*xdot;                             % Solve velocitities via RMRC
                qmatrix(rowIdx + j,:) =  qmatrix(rowIdx + j - 1,:) + deltaT * qdot';                   % Update next joint state
            end
        end
    % end
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
% myUR3 = UR3(transl(1.02, -0.01, 0));
% steps = 50;
% calculateUr3Trajectory(myUR3, steps);     
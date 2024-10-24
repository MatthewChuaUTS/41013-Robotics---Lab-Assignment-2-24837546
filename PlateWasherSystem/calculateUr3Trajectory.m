function [ur3TrajectoryQmatrix] = calculateUr3Trajectory(myUR3, steps)
    if nargin < 2
        steps = 50;
    end
    trSteps = {transl([0.747,-0.049,0.072]) ... % sponge
        , transl([-0.2695, -0.07565, 0.3898]) ...  % srub position
        , transl([-0.516, 0.1004, 0.262]) ...  % srub position
        , transl([-0.516, 0.1004, 0.262]) ...  % srub position
        , transl([-0.516, 0.1004, 0.262]) ...  % srub position
        , transl([-0.516, 0.1004, 0.262]) ...  % srub 
        , transl([-0.516, 0.1004, 0.262]) ...  % srub 
        , transl([0.747,-0.049,0.072]) ...      % sponge
        , transl([0.1674, -0.1163, 0.4049]) ...  % hold
        , transl([0.1674, -0.1163, 0.4049]) ...  % hold
        , transl([0.1674, -0.1163, 0.4049]) ...  % hold
        , transl([-0.0735, 0.03071, 0.2057]) ...   % dunk
        , transl([-0.2612, -0.09066, 0.5106]) ...   % up
        , transl([-0.08535, -0.1123, 0.5269])};  % place

    % trSteps = {transl([0.747,-0.049,0.072]) ... % sponge
    %     , transl([-0.2695, -0.07565, 0.3898]) ...  % srub position
    %     , transl([-0.2695, -0.07565, 0.3898]) ...  % srub position
    %     , transl([-0.2695, -0.07565, 0.3898]) ...  % srub position
    %     , transl([-0.2695, -0.07565, 0.3898]) ...  % srub position
    %     , transl([-0.2695, -0.07565, 0.3898]) ...  % srub 
    %     , transl([-0.2695, -0.07565, 0.3898]) ...  % srub 
    %     , transl([0.747,-0.049,0.072]) ...      % sponge
    %     , transl([0.801, -0.1163, 0.4049]) ...  % hold
    %     , transl([0.801, -0.1163, 0.4049]) ...  % hold
    %     , transl([0.801, -0.1163, 0.4049]) ...  % hold
    %     , transl([-0.839, -0.273, 0.014]) ...   % dunk
    %     , transl([0.842, 0.26, 0.327]) ...   % up
    %     , transl([1.219, 0.322, 0.191])};  % place

  %    qWaypoints = [[0, -pi/2, 0, 0, -pi/2, 0];
  %       [-0.2778, -0.1192, 1.227, 0.463, -1.5709, -1.293];  % sponge
  %       [-0.7190, -1.0296, 0.8819, 1.7187, -1.5708, -0.8520];
  %       deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]);
  %       [-1.3482, 0.5075, -0.0001, 1.0893, -1.5499, -0.2229];  % dunk
  %       [-0.1932, -0.2344, 0.0003, 2.2783, -1.4885, -1.3972]; % up
  %       deg2rad([0, -57.1, 131, -163, -90, 0]);
  %       deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
  %       deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
  %       deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]);
  %       deg2rad([-71.4, -11.4, 65.7, -146, -90, -32.1]);
  %       deg2rad([-134, -78.3, 102, 250, -93.9, 110])];
  % 
  % myUR3.model.ikcon(transl([0.842, 0.26, 0.327]),[-1.3482, 0.5075, -0.0001, 1.0893, -1.5499, -0.2229])
  % myUR3.model.plot([-0.1932, -0.2344, 0.0003, 2.2783, -1.4885, -1.3972]);
  % myUR3.model.teach([-0.1932, -0.2344, 0.0003, 2.2783, -1.4885, -1.3972]);

    qmatrix = zeros((length(trSteps) - 1) * steps, 6);
    qWaypoints = zeros((length(trSteps)) * steps, 6);  
    deltaT = 0.05; 
    UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];
    myUR3.model.plot(UR3CurrentJointPosition);
    
    % Generate the joint angle using ikcon
    for i = 1:length(trSteps)
        qWaypoints(i, :) = myUR3.model.ikcon(trSteps{i}, UR3CurrentJointPosition);
        UR3CurrentJointPosition = qWaypoints(i, :); 
    end

    rowIdx = 1;  
    for i = 1:(length(trSteps) - 1) 
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
        if i >= 7 && i <= 14
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

        if i >= 4 && i <= 6  
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
% Ur3TrajectoryQmatrix = calculateUr3Trajectory(myUR3, steps); 
% 
% myUR3.model.delay = 0;
% for j = 1:size(Ur3TrajectoryQmatrix, 1)
%     myUR3.model.animate(Ur3TrajectoryQmatrix(j, :));
%     drawnow(); 
%     pause(0.01);  
% end
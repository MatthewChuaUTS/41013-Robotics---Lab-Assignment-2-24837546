function [niryoTrajectoryQmatrix] = calculateNiryoTrajectory(myNiryoOne, steps)
    if nargin < 2
        steps = 50;
    end
    trSteps = {transl([0.22,0.061,0.195]) ... % plate
        , transl([0.358,0.196,0.244]) ... % point2
        , transl([0.358,0.196,0.005]) ... % dunk
        , transl([0.358,0.196,0.406]) ... % point4  RMRC in lab happens for same axis, goes straight, is it kay if the points are short?
        , transl([0.678,0.196,0.406]) ... % hold
        , transl([0.678,0.196,0.406]) ... % hold
        , transl([0.678,0.196,0.406]) ... % hold
        , transl([0.678,0.196,0.406]) ... % hold
        , transl([0.678,0.196,0.406]) ... % hold
        , transl([0.747,-0.049,0.072]) ... % sponge
        , transl([0.655,0.074,0.457]) ... % scrub
        , transl([0.655,0.074,0.457]) ... % srub
        , transl([0.747,-0.049,0.072]) ... % sponge
        , transl([0.22,0.061,0.195])};

    % Preallocate qmatrix for 600 rows (12 segments x 50 steps) and 6 columns
    % (more rows for this one, will eventually made it equal to ur3)
    qmatrix = zeros((length(trSteps) - 1) * steps, 6);
    qWaypoints = zeros((length(trSteps)) * steps, 6); 
    deltaT = 0.05; 
    niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];

    % Generate the joint angle using ikcon
    for i = 1:length(trSteps)
        qWaypoints(i, :) = myNiryoOne.model.ikcon(trSteps{i}, niryoOneCurrentJointPosition);
        niryoOneCurrentJointPosition = qWaypoints(i, :); 
    end
    
    % Generate the trajectory and store it in the preallocated matrix
    rowIdx = 1;  
    for i = 1:(length(trSteps) - 1) 
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
        if i >= 1 && i <= 8 
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
                J = myNiryoOne.model.jacob0(qmatrix(rowIdx + j - 1, :));            % Get the Jacobian at the current state
                J = J(1:2,:);                           % Take only first 2 rows
                qdot = pinv(J)*xdot;                             % Solve velocitities via RMRC
                qmatrix(rowIdx + j,:) =  qmatrix(rowIdx + j - 1,:) + deltaT * qdot';                   % Update next joint state
            end
        end
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;

        if i >= 9 && i <= 10 
            for j = 1:steps
                traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps);  
            end
        end
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;
        rowIdx = rowIdx + steps;
        disp(rowIdx);
    end
    
    niryoTrajectoryQmatrix = qmatrix;

    % Animate the NiryoOne robot along the generated trajectory
    % myNiryoOne.model.delay = 0;
    % for j = 1:size(qmatrix, 1)
    %     myNiryoOne.model.animate(qmatrix(j, :));
    %     drawnow();  
    %     % pause(0.01); 
    % end
end

% Probably also insert velocity acceleration from Lab4Q2-3 to check if RMRC
% graph working
% figure(1)
    % for i = 1:6
    %     subplot(3,2,i)
    %     plot(qmatrix(:,i),'k','LineWidth',1)
    %     title(['Joint ', num2str(i)])
    %     xlabel('Step')
    %     ylabel('Joint Angle (rad)')
    % end

% Call function to test it's running
% environment();
% steps = 50;  
% niryoTrajectoryQmatrix = calculateNiryoTrajectory(myNiryoOne, steps); 
% Ur3TrajectoryQmatrix = calculateUr3Trajectory(myUR3, steps); 
% 
% myNiryoOne.model.delay = 0;
% myUR3.model.delay = 0;
% for j = 1:size(qmatrix, 1)
%     myNiryoOne.model.animate(niryoTrajectoryQmatrix(j, :));
%     myUR3.model.animate(Ur3TrajectoryQmatrix(j, :));
%     drawnow(); 
%     pause(0.01);  
% end


% myNiryoOne = niryoOne(transl(0.54, -0.01, 0)); 
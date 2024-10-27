function [niryoTrajectoryQmatrix] = calculateNiryoTrajectory(myNiryoOne, steps, faces, vertex, faceNormals) %returnOnceFound
    if nargin < 2
        steps = 50;
    end
    if nargin < 3
        % Default values if faces, vertex, and faceNormals are not provided
        centerpnt = [1,1,1];
        side = 0.2;
        plotOptions.plotFaces = true;
        [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
        axis equal;
    end 
    % if nargin < 6
    %     % Default value
    %     % returnOnceFound = true;
    % end

    trSteps = {transl([0.22,0.061,0.195]) ... % plate 1
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
        , transl([0.3203,-0.01,0.423]) * trotx(-pi/2)}; % reset 18

    % Preallocate qmatrix for 600 rows (12 segments x 50 steps) and 6 columns
    % (more rows for this one, will eventually made it equal to ur3)
    qmatrix = zeros((length(trSteps) - 1) * steps, 6);
    qWaypoints = zeros((length(trSteps)) * steps, 6); 
    deltaT = 0.05; 
    niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];
    myNiryoOne.model.plot(niryoOneCurrentJointPosition);  % this is for
    % function testing
 
    % Generate the joint angle using ikcon
    for i = 1:length(trSteps)
        qWaypoints(i, :) = myNiryoOne.model.ikcon(trSteps{i}, niryoOneCurrentJointPosition);
        niryoOneCurrentJointPosition = qWaypoints(i, :); 
    end

    % Generate the trajectory and store it in the preallocated matrix
    rowIdx = 1;  
%     for i = 1:(length(trSteps) - 1) 
%         traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
%         if (i >= 1 && i <= 10) || (i > 12)
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
%                 xdot = (x(:,j+1) - x(:,j))/deltaT;                             % Calculate velocity at discrete time step
%                 J = myNiryoOne.model.jacob0(qmatrix(rowIdx + j - 1, :));            % Get the Jacobian at the current state
%                 J = J(1:2,:);                           % Take only first 2 rows
%                 qdot = pinv(J)*xdot;                             % Solve velocitities via RMRC
%                 qmatrix(rowIdx + j,:) =  qmatrix(rowIdx + j - 1,:) + deltaT * qdot';                   % Update next joint state
%             end
%         end
%         qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;
% 
%         if i >= 11 && i <= 12 
%             for j = 1:steps
%                 traj(j, 6) = traj(j, 6) + deg2rad(360 * j / steps);  
%             end
%         end
%         qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;
%         rowIdx = rowIdx + steps;
%         disp(rowIdx);
%     end
% 
%     niryoTrajectoryQmatrix = qmatrix;
% 
%     % myNiryoOne.model.delay = 0;
% 
%         % if CollisionIsDetected
%         %     disp('Collision detected at step: ');
%         %     disp(j);
%         % end
% 
%     myNiryoOne.model.delay = 0;
%     myNiryoOne.model.animate(niryoTrajectoryQmatrix(j,:));
%     % niryoOneCurrentJointPosition = [niryoTrajectoryQmatrix(j,:); niryoTrajectoryQmatrix(j,:)];                                                       % from q1 to q2
%     isCollision = true;
%     checkedTillWaypoint = 1;                                                    % distance robot moved without collision 1 = q1
% 
%     while(isCollision)
%     startWaypoint = checkedTillWaypoint;
%         for j = startWaypoint:size(niryoTrajectoryQmatrix, 1)-1  % from startwaypoint to last one
% 
%             qWaypoints = [niryoTrajectoryQmatrix(j,:); niryoTrajectoryQmatrix(j + 1,:)];
%             % forming array
%             niryoTrajectoryQMatrixJoin = InterpolateWaypointRadians(qWaypoints(j:j+1,:),deg2rad(10));        
%             if ~IsCollision(myNiryoOne, niryoTrajectoryQMatrixJoin,faces, vertex, faceNormals)
%                 % QMatrix with interpolated QMatrix
%                 % niryoTrajectoryQmatrix = [niryoTrajectoryQmatrix(j,:); niryoTrajectoryQMatrixJoin];
% 
%                 % print size before concatenation 
%                 disp('Size of niryoTrajectoryQmatrix(j,:) before concatenation:');
%                 disp(size(niryoTrajectoryQmatrix(j,:)));
%                 disp('Size of niryoTrajectoryQMatrixJoin before concatenation:');
%                 disp(size(niryoTrajectoryQMatrixJoin));
% 
%                 % if size(niryoTrajectoryQmatrix(j,:), 2) == size(niryoTrajectoryQMatrixJoin, 2)
%                 %     niryoTrajectoryQmatrix = [niryoTrajectoryQmatrix(1:j,:); niryoTrajectoryQMatrixJoin; niryoTrajectoryQmatrix(j+1:end,:)];
%                 % else
%                 %     disp('Inconsistent dimensions. Cannot concatenate.');
%                 % end
% 
%                 myNiryoOne.model.animate(niryoTrajectoryQMatrixJoin);
% 
%                 isCollision = false;
%                 checkedTillWaypoint = i + 1;
% 
%                 % Now try and join to the final goal (q2)
%                 niryoTrajectoryQMatrixJoin = InterpolateWaypointRadians([niryoTrajectoryQmatrix(end,:); niryoTrajectoryQmatrix(j+1,:)],deg2rad(10));
% 
%                 if ~IsCollision(myNiryoOne, niryoTrajectoryQMatrixJoin, faces, vertex, faceNormals)
%                     niryoTrajectoryQmatrix = [niryoTrajectoryQmatrix(1:j,:); niryoTrajectoryQMatrixJoin];
%                     % Reached goal without collision, so break out
%                     break;
%                 end
%             else
%                 % Randomly pick a pose that is not in collision
%                 qRand = (2 * rand(1, 6) - 1) * pi; % Random joint angles within [-pi, pi]
%                 while IsCollision(myNiryoOne, qRand, faces, vertex, faceNormals)
%                     qRand = (2 * rand(1, 6) - 1) * pi; % Random joint angles within [-pi, pi]
%                 end
%                 qWaypoints =[qWaypoints(1:j,:); qRand; qWaypoints(j+1:end,:)];
%                 isCollision = true;
%                 break;
%             end
%         end
%     end
% 
%     myNiryoOne.model.animate(qmatrix(j, :));
%     drawnow();  
%     pause(0.01); 
% end
    
    for i = 1:(length(trSteps) - 1)
        traj = jtraj(qWaypoints(i, :), qWaypoints(i + 1, :), steps);
        qmatrix(rowIdx:rowIdx + steps - 1, :) = traj;
        rowIdx = rowIdx + steps;
    end
    
    % Collision-Avoidance Logic
    isCollision = true;
    checkedTillWaypoint = 1;
    niryoTrajectoryQmatrix = qmatrix; % Use initially generated trajectory
    
    while isCollision
        startWaypoint = checkedTillWaypoint;
        for j = startWaypoint:size(niryoTrajectoryQmatrix, 1) - 1
            % Form and interpolate waypoint
            qWaypoints = [niryoTrajectoryQmatrix(j, :); niryoTrajectoryQmatrix(j + 1, :)];
            niryoTrajectoryQMatrixJoin = InterpolateWaypointRadians(qWaypoints, deg2rad(10));
            
            % Check for collision
            if ~IsCollision(myNiryoOne, niryoTrajectoryQMatrixJoin, faces, vertex, faceNormals)
                % If no collision, update trajectory and animate
                disp('Updating trajectory with interpolated path:');
                disp(size(niryoTrajectoryQmatrix(j, :)));
                disp(size(niryoTrajectoryQMatrixJoin));
                
                niryoTrajectoryQmatrix = [niryoTrajectoryQmatrix(1:j, :); niryoTrajectoryQMatrixJoin; niryoTrajectoryQmatrix(j+1:end, :)];
                myNiryoOne.model.animate(niryoTrajectoryQMatrixJoin);

                isCollision = false;
                checkedTillWaypoint = j + 1;

                % Attempt to join to the final goal
                niryoTrajectoryQMatrixJoin = InterpolateWaypointRadians([niryoTrajectoryQmatrix(end, :); niryoTrajectoryQmatrix(j + 1, :)], deg2rad(10));
                if ~IsCollision(myNiryoOne, niryoTrajectoryQMatrixJoin, faces, vertex, faceNormals)
                    niryoTrajectoryQmatrix = [niryoTrajectoryQmatrix(1:j, :); niryoTrajectoryQMatrixJoin];
                    break; % Break if no collision to final goal
                end
            else
                % Handle collision: Random pose not in collision
                qRand = (2 * rand(1, 6) - 1) * pi;
                while IsCollision(myNiryoOne, qRand, faces, vertex, faceNormals)
                    qRand = (2 * rand(1, 6) - 1) * pi;
                end
                qWaypoints = [qWaypoints(1:j, :); qRand; qWaypoints(j+1:end, :)];
                isCollision = true;
                break;
            end
        end
    end
    
    % Final Animation
    for k = 1:size(niryoTrajectoryQmatrix, 1)
        myNiryoOne.model.animate(niryoTrajectoryQmatrix(k, :));
        drawnow();
        pause(0.01);
    end
end


%% Call function to test it's running
%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

    u = triangleVerts(2,:) - triangleVerts(1,:);
    v = triangleVerts(3,:) - triangleVerts(1,:);

    uu = dot(u,u);
    uv = dot(u,v);
    vv = dot(v,v);

    w = intersectP - triangleVerts(1,:);
    wu = dot(w,u);
    wv = dot(w,v);

    D = uv * uv - uu * vv;

    % Get and test parametric coords (s and t)
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
        result = 0;
        return;
    end

    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
        result = 0;
        return;
    end

    result = 1;                      % intersectP is in Triangle
end


%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function CollisionIsDetected = IsCollision(myNiryoOne,niryoTrajectoryQmatrix,faces,vertex,faceNormals,returnOnceFound)
    if nargin < 6
        returnOnceFound = true;
    end
    CollisionIsDetected = false;

    for qIndex = 1:size(niryoTrajectoryQmatrix,1)
        % Get the transform of every joint (i.e. start and end of every link)
        tr = GetLinkPoses(niryoTrajectoryQmatrix(qIndex,:), myNiryoOne);

        % Go through each link and also each triangle face
        for i = 1 : size(tr,3)-1    
            for faceIndex = 1:size(faces,1)
                vertOnPlane = vertex(faces(faceIndex,1)',:);
                [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 

                if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                    plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                    disp('Intersection');
                    CollisionIsDetected = true;
                    if returnOnceFound
                        return 
                    end
                end
            end    
        end
    end
end


%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
function [ transforms ] = GetLinkPoses(q, myNiryoOne)
    links = myNiryoOne.model.links;
    transforms = zeros(4, 4, length(links) + 1);
    transforms(:,:,1) = myNiryoOne.model.base;

    for i = 1:length(links)
        L = links(1,i);

        current_transform = transforms(:,:, i);

        current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
        transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
        transforms(:,:,i + 1) = current_transform;
    end
end


%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function niryoTrajectoryQmatrix = FineInterpolation(q1,q2,maxStepRadians)
    if nargin < 3
        maxStepRadians = deg2rad(1);
    end

    steps = 2;
    while true
        traj = jtraj(q1,q2,steps);

        if all(abs(diff(traj)) < maxStepRadians)
            break;
        end
        steps = steps + 1;
    end
    % check jtraj has correct matrix size
    niryoTrajectoryQmatrix = jtraj(q1,q2,steps);

    if size(niryoTrajectoryQmatrix, 2) ~= 6
        error('Unexpected output dimension from FineInterpolation. Ensure 6 columns.');
    end
end


%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1 % amking array for qmatrix
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end


% %% GenerateRandConfig
% function qRand = GenerateRandConfig(myNiryoOne)
%     qRand = (2 * rand(1, 6) - 1) * pi; % Random joint angles within [-pi, pi]
% end


%% call test
% myNiryoOne = niryoOne(transl(0.54, -0.01, 0));
% steps = 50;
% 
% centerpnt = [0.5, 0.3, 0.3];  
% sideLength = 0.3; 
% plotOptions.plotFaces = true;
% [vertex, faces, faceNormals] = RectangularPrism(centerpnt - sideLength/2, centerpnt + sideLength/2, plotOptions);
% % axis ([0.5, 0.1, -0.7, 0.7, 0, 1]);
% 
% niryoTrajectoryQmatrix = calculateNiryoTrajectory(myNiryoOne, steps, faces, vertex, faceNormals);
% myNiryoOne.model.delay = 0;
% for j = 1:size(niryoTrajectoryQmatrix, 1)
%     myNiryoOne.model.animate(niryoTrajectoryQmatrix(j, :));
%     drawnow(); 
%     pause(0.01);  
% end



% %% bury
% 
% % Probably also insert velocity acceleration from Lab4Q2-3 to check if RMRC
% % graph working
% % figure(1)
%     % for i = 1:6
%     %     subplot(3,2,i)
%     %     plot(qmatrix(:,i),'k','LineWidth',1)
%     %     title(['Joint ', num2str(i)])
%     %     xlabel('Step')
%     %     ylabel('Joint Angle (rad)')
%     % end
% 

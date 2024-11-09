% Initialize the robot
% environment();
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));

% Define the obstacle (adjust as needed for your environment)
% centerpnt = [0.2,0,0.3]; % intersect fully
% centerpnt = [0.2,0.06,0.3];
centerpnt = [-0.2,0.06,0.3];
side = 0.3;
plotOptions.plotFaces = true;
[vertex, faces, faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2, plotOptions);

trSteps = {transl([0.22,0.061,0.195]) ... % plate 1
        , transl([0.22,0.196,0.244]) ... % point2.1 2
        , transl([0.358,0.196,0.244]) ... % point2 3
        , transl([0.358,0.196,-0.005]) ... % dunk 4
        , transl([0.358,0.196,0.406]) ... % point4 5  RMRC in lab happens for same axis, goes straight, is it kay if the points are short?
        , transl([0.6975,0.263,0.3915]) ... % hold 6
        , transl([0.6975,0.263,0.3915]) ... % hold 7
        , transl([0.6975,0.263,0.3915]) ... % hold 8
        , transl([0.6975,0.263,0.3915]) ... % hold 9
        , transl([0.6975,0.263,0.3915]) ... % hold 10
        , transl([0.747,-0.049,0.072]) ... % sponge 11
        , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % scrub position 12
        , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % scrub 13
        , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % srub 14
        , transl([0.726,0.1495,0.4695]) ... % scrub position 15
        , transl([0.747,-0.049,0.072]) ... % sponge 16
        , transl([0.3203,-0.01,0.423]) * trotx(-pi/2) ... % reset 17
        , transl([0.3203,-0.01,0.423]) * trotx(-pi/2)}; % reset 18
        

% Convert trSteps to qWaypoints
qWaypoints = zeros(length(trSteps), 6); % 18x6
for i = 1:length(trSteps)
    qWaypoints(i, :) = myNiryoOne.model.ikcon(trSteps{i});
end

% Initialize variables
qMatrix = [];

for i = 1:length(trSteps)-1
    q1 = qWaypoints(i, :);
    q2 = qWaypoints(i + 1, :);
% Initial and goal configurations
% q1 = myNiryoOne.model.getpos();
% q2 = qWaypoints(end, :);


    isCollision = true;
    checkedTillWaypoint = 1;
    maxAttempts = 100; % Maximum number of attempts to find a collision-free path
    
    attemptCount = 0;
    while (isCollision && attemptCount < maxAttempts)
        startWaypoint = checkedTillWaypoint;
        for i = startWaypoint:size(qWaypoints,1)-1
            qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:), deg2rad(30));
            if ~IsCollision(myNiryoOne, qMatrixJoin, faces, vertex, faceNormals)
                qMatrix = [qMatrix; qMatrixJoin];
                % myNiryoOne.model.animate(qMatrixJoin);
                isCollision = false;
                checkedTillWaypoint = i+1;
                
                % qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2], deg2rad(5));   % q3
                % if ~IsCollision(myNiryoOne, qMatrixJoin, faces, vertex, faceNormals)
                %     qMatrix = [qMatrix; qMatrixJoin];
                %     break;
                % end
            else
                qRand = (2 * rand(1,6) - 1) .* myNiryoOne.model.qlim(:,2)';
                attempts = 0;
                while IsCollision(myNiryoOne, qRand, faces, vertex, faceNormals) && attempts < 50
                    qRand = (2 * rand(1,6) - 1) .* myNiryoOne.model.qlim(:,2)';
                    attempts = attempts + 1;
                end
                if attempts >= 50
                    disp('Unable to find a collision-free random pose');
                    break;
                end
                qWaypoints = [qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                isCollision = true;
                break;
            end
        end
        attemptCount = attemptCount + 1;
    end
    
    if attemptCount >= maxAttempts
        disp('Unable to find a collision-free path after maximum attempts');
    else
        disp('Collision-free path found');
        myNiryoOne.model.animate(qMatrix);
    end
end

% for j = 1:size(qMatrix, 1)
%     myNiryoOne.model.animate(qMatrix(j, :));
%     drawnow(); 
%     pause(0.01);  
% end

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


% testingtestercollision figuring qmatrix and qmatrixjoin to figure q1 and
% q2

% while (isCollision && attemptCount < maxAttempts)
%     startWaypoint = checkedTillWaypoint;
%     for i = startWaypoint:size(qWaypoints,1)-1
%         % qMatrixJoin for q1 = myNiryoOne.model.getpos()
%         % 0.5006, -0.6405, -1.5708, -3.0543, 0.6410, -2.5711
%         % -0.0313   -0.6405   -1.5708   -0.0001   -0.6412   -0.0315
% 
%         % qMatrix
%         % 0.2706   -0.6405   -1.5708    0.0003   -0.6393    0.2711
%         %    0.5006   -0.6405   -1.5708   -3.0542    0.6409   -2.5710
%         %    0.5006   -0.6405   -1.5708   -3.0543    0.6410   -2.5711
%         % -0.0313   -0.6405   -1.5708   -0.0001   -0.6412   -0.0315
% 
%         qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:), deg2rad(5));
%         if ~IsCollision(myNiryoOne, qMatrixJoin, faces, vertex, faceNormals)
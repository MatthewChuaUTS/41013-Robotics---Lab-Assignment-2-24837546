axis(0, 1.5, -1, 1, 0, 0.5);
hold on;
% Set up Robot 1
niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];  
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));  

% Set up Robot 2
% NOTE, SET UR3 QLIM JOINT 2 TO 30 AND 150 DEG, WHATEVER THAT IS IN RAD
UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];  
myUR3 = UR3(transl(1.02, -0.01, 0));                                              

%% ellipsoid wrap joints of Robot 2  
centerPoint = [0,0,0];
radii = [0.2,0.1,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:7
    myUR3.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    myUR3.model.faces{i} = delaunay(myUR3.model.points{i});    
    warning on;
end

myUR3.model.plot3d(UR3CurrentJointPosition); 
camlight
myUR3.model.teach;

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end

% ellipsoid wrap joints of Robot 1
centerPoint = [0,0,0];
radii = [0.2,0.1,0.1];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:7
    myNiryoOne.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    myNiryoOne.model.faces{i} = delaunay(myNiryoOne.model.points{i});    
    warning on;
end

myNiryoOne.model.plot3d(niryoOneCurrentJointPosition);
camlight
% myNiryoOne.model.teach;

%% collision ellipse to ellipse (case 2 = robot to robot)
while true
    % Get current joint positions from both robots
    q1 = myNiryoOne.model.getpos(); 
    q2 = myUR3.model.getpos();      

    % % Update transformation matrices for Robot 1 (NiryoOne)
    trNiryo = zeros(4,4,myNiryoOne.model.n+1);
    trNiryo(:,:,1) = myNiryoOne.model.base;
    for i = 1:myNiryoOne.model.n
        trNiryo(:,:,i+1) = trNiryo(:,:,i) * trotz(q1(i)) * transl(0,0,myNiryoOne.model.links(i).d) * ...
                           transl(myNiryoOne.model.links(i).a,0,0) * trotx(myNiryoOne.model.links(i).alpha);
    end
    
    % Update transformation matrices for Robot 2 (UR3)
    trUR3 = zeros(4,4,myUR3.model.n+1);
    trUR3(:,:,1) = myUR3.model.base;
    for i = 1:myUR3.model.n
        trUR3(:,:,i+1) = trUR3(:,:,i) * trotz(q2(i)) * transl(0,0,myUR3.model.links(i).d) * ...
                         transl(myUR3.model.links(i).a,0,0) * trotx(myUR3.model.links(i).alpha);
    end

    % % Collision Check: Compare each ellipsoid of Robot 1 against Robot 2
    % for i = 1:size(trNiryo,3)
    %     % Transform points from NiryoOne to UR3's coordinate frame
    %     niryoPointsTransformed = [inv(trUR3(:,:,i)) * [myNiryoOne.model.points{i}, ones(size(myNiryoOne.model.points{i},1),1)]']';
    %     niryoUpdatedPoints = niryoPointsTransformed(:,1:3);
    % 
    %     % Algebraic distance between points and ellipsoid in UR3
    %     algebraicDist = GetAlgebraicDist(niryoUpdatedPoints, centerPoint, radii);
    %     pointsInside = find(algebraicDist < 1);
    % 
    %     % Display collision information
    %     if ~isempty(pointsInside)
    %         disp(['Collision detected between NiryoOne and UR3 at joint ', num2str(i)]);
    %     end
    % end
    
    % Collision Check: Compare each ellipsoid of Robot 2 against Robot 1
    for i = 1:size(trUR3,3)
        % Transform points from UR3 to NiryoOne's coordinate frame
        ur3PointsTransformed = [inv(trNiryo(:,:,i)) * [myUR3.model.points{i}, ones(size(myUR3.model.points{i},1),1)]']';
        ur3UpdatedPoints = ur3PointsTransformed(:,1:3);
        
        % Algebraic distance between points and ellipsoid in NiryoOne
        algebraicDist = GetAlgebraicDist(ur3UpdatedPoints, centerPoint, radii);
        pointsInside = find(algebraicDist < 1);
        
        % Display collision information
        if ~isempty(pointsInside)
            disp(['Collision detected between UR3 and NiryoOne at joint ', num2str(i)]);
        end
    end
    
    pause(1); % To allow smooth real-time collision checking
end
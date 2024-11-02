% NOTE, SET UR3 QLIM JOINT 2 TO 30 AND 150 DEG, WHATEVER THAT IS IN RAD
%% Ellipsoid wrap joints of Robot 2 UR3
function collisionCheck(myRobot)
    % env = environment();
    myRobotCurrentJointPosition = myRobot.model.getpos();    
    
    centerPoint = [0,0,0];
    radii = [0.1,0.5,0.5];
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    for i = 2:7
        myRobot.model.points{i} = [X(:),Y(:),Z(:)];
        warning off
        myRobot.model.faces{i} = delaunay(myRobot.model.points{i});    
        warning on;
    end
    
    myRobot.model.plot3d(myRobotCurrentJointPosition); 
    myRobot.model.teach(myRobotCurrentJointPosition);
    % Sink pointcloud
    [X,Y] = meshgrid(0:0.075:1.5,-0.3:0.04:0.5);
    Z = zeros(size(Y));
    oneSideOfCube_h = surf(X,Y,Z);
    
    cubePoints = [X(:),Y(:),Z(:)];      
    cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
    
    % Robot joints update information
    UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0]; 
    tr = zeros(4,4,myRobot.model.n+1);
    tr(:,:,1) = myRobot.model.base;
    L = myRobot.model.links;
    for i = 2 : myRobot.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(UR3CurrentJointPosition(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
    
    % Go through each ellipsoid
    for i = 2: size(tr,6)
        cubePointsAndOnes = [inv(tr(:,:,i)) * [cubePoints,ones(size(cubePoints,1),1)]']';
        updatedCubePoints = cubePointsAndOnes(:,1:3);
        algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
        pointsInside = find(algebraicDist < 1);
        disp(['There are ', num2str(size(pointsInside,1)),' points collided with the ',num2str(i),'th ellipsoid']);
    end
end

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

    algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end





% do this later
%% ellipsoid wrap joints of Robot 1
centerPoint = [0,0,0];
radii = [0.05,0.1,0.05];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:7
    myNiryoOne.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    myNiryoOne.model.faces{i} = delaunay(myNiryoOne.model.points{i});    
    warning on;
end

myNiryoOne.model.plot3d(niryoOneCurrentJointPosition);
camlight
myNiryoOne.model.teach;
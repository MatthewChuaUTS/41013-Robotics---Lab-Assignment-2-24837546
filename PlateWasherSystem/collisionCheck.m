axis(-0.8, 0.8, -0.8, 0.8, 0, 0.5);
% Set up Robot 1
niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];  
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));  
myNiryoOne.model.animate(niryoOneCurrentJointPosition);  

% Set up Robot 2
% NOTE, SET UR3 QLIM JOINT 2 TO 30 AND 150 DEG, WHATEVER THAT IS IN RAD
UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];  
myUR3 = UR3(transl(1.02, -0.01, 0));                                              
myUR3.model.animate(UR3CurrentJointPosition); 

% ellipsoid wrap joints of Robot 2  
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
% axis equal
camlight
myUR3.model.teach;
% 
% % Pointcloud cube
% [Y,Z] = meshgrid(-0.75:0.05:0.75,-0.75:0.05:0.75);
% sizeMat = size(Y);
% X = repmat(0.75,sizeMat(1),sizeMat(2));
% oneSideOfCube_h = surf(X,Y,Z);
% 
% cubePoints = [X(:),Y(:),Z(:)];
% 
% % Make a cube by rotating the single side
% cubePoints = [ cubePoints ...
%              ; cubePoints * rotz(pi/2)...
%              ; cubePoints * rotz(pi) ...
%              ; cubePoints * rotz(3*pi/2) ...
%              ; cubePoints * roty(pi/2) ...
%              ; cubePoints * roty(-pi/2)];         
% 
% % Plot the cube's point cloud         
% cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');
% 
% tr = myUR3.model.fkine(UR3CurrentJointPosition).T;
% cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
% updatedCubePoints = cubePointsAndOnes(:,1:3);
% algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
% pointsInside = find(algebraicDist < 1);
% disp(['There are now ', num2str(size(pointsInside,1)),' points inside']);
% 
% function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
% 
% algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
%               + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
%               + ((points(:,3)-centerPoint(3))/radii(3)).^2;
% end

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
% axis equal
camlight
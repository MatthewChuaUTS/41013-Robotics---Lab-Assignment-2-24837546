% function [ur3TrajectoryQmatrix] = calculateUr3Trajectory(myUR3, totalSteps)
% end

clf;
clear all;

env = environment();  %%
myUR3 = UR3(transl(1.02, -0.01, 0));

UR3InitialJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];                                                
myUR3.model.animate(UR3InitialJointPosition); 

qWaypoints = [UR3InitialJointPosition ...
    ; deg2rad([0, -57.1, 131, -163, -90, 0]) ...
    ; deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]) ...
    ; deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]) ...
    ; deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]) ...
    ; deg2rad([-25.7, -88.6, 88.6, -171, -151, 0]) ...
    ; deg2rad([0, -57.1, 131, -163, -90, 0]) ...
    ; deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]) ...
    ; deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]) ...
    ; deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]) ...
    ; deg2rad([-68.6, -82.9, 22.9, -42.9, -68.6, 0]) ...
    ; deg2rad([-134, -74.3, 111, -12.9, -90, -32.1]) ...
    ; UR3InitialJointPosition];

% qMatrix = InterpolateWaypointRadians(qWaypoints,deg2rad(5));
% robot.animate(qMatrix);

for i = 2:length(qWaypoints)
    qmatrix = jtraj(qWaypoints(i-1),qWaypoints(i),50);
    myUR3.model.animate(qmatrix);
end



%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
if nargin < 3
    maxStepRadians = deg2rad(1);
end
    
steps = 2;
while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
    steps = steps + 1;
end
qMatrix = jtraj(q1,q2,steps);
end

%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
if nargin < 2
    maxStepRadians = deg2rad(1);
end

qMatrix = [];
for i = 1: size(waypointRadians,1)-1
    qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
end
end






%% bury
% myUR3.model.teach(UR3CurrentJointPosition

% T1 = [eye(3) [0.65 0.071 0.474]'; zeros(1,3) 1];  
% % targettr = endEffectorTr * transl([0.757,0.071,0.474]);
% NcircularMotion = myNiryoOne.model.ikcon(T1, niryoOneCurrentJointPosition);
% myNiryoOne.model.animate(NcircularMotion);  
% myNiryoOne.model.teach(NcircularMotion);

clf;
clear all;

env = environment();

% Set up Robot 1
niryoOneCurrentJointPosition = [deg2rad(-125), deg2rad(12.9), deg2rad(-37.5), 0, 0, 0];  
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));  
myNiryoOne.model.animate(niryoOneCurrentJointPosition);  
% myNiryoOne.model.teach(niryoOneCurrentJointPosition);c

% Set up Roboclet 2

% NOTE, SET UR3 QLIM JOINT 2 TO 30 AND 150 DEG, WHATEVER THAT IS IN RAD
UR3CurrentJointPosition = [deg2rad(-45.7), deg2rad(-82.9), deg2rad(94.3), deg2rad(-186), deg2rad(-134), deg2rad(0)];  
myUR3 = UR3(transl(1.02, -0.01, 0));                                              
myUR3.model.animate(UR3CurrentJointPosition); 
myUR3.model.teach(UR3CurrentJointPosition);

tr = myUR3.model.fkine(UR3CurrentJointPosition).T;
circularMotion = myUR3.model.ikine(tr, 'UR3CurrentJointPosition', 'mask', [0,0,0,1,1,0], 'alpha');

% piggy = RobotPiggy(transl(0.8, -1.3, -0.7971));

sensorTrip = 0;
% nothing changed - just note SerialLink
% piggy.movePiggy(transl(0.8, -1, -0.7971))
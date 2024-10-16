clf;
clear all;

env = environment();

% Set up Robot 1
niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];  
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));  
myNiryoOne.model.animate(niryoOneCurrentJointPosition);  

% Set up Robot 2

% NOTE, SET UR3 QLIM JOINT 2 TO 30 AND 150 DEG, WHATEVER THAT IS IN RAD
UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];  
myUR3 = UR3(transl(1.02, -0.01, 0));                                              
myUR3.model.animate(UR3CurrentJointPosition); 

piggy = RobotPiggy(transl(0.8, -1.3, -0.7971));

sensorTrip = 0;
% nothing changed - just note SerialLink
% piggy.movePiggy(transl(0.8, -1, -0.7971))
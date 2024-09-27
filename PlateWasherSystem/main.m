environment();
currentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];  
myUR3 = UR3(transl(1.02, -0.01, 0));                                               % Places the robot at that point
myUR3.model.animate(currentJointPosition);    

link(1) = Link([0      0.08       0       -pi/2]); 
link(2) = Link([0      0          0.21    0]);
link(3) = Link([0      0.03       0.0415  -pi/2]);
link(4) = Link([0      -0.18      0       pi/2]);
link(5) = Link([0      -0.055     0.0237  pi/2]);
link(6) = Link([0      0          0       0]);

link(1).qlim = [-175 175]*pi/180;
link(2).qlim = [-90 36.7]*pi/180;
link(3).qlim = [-80 90]*pi/180;
link(4).qlim = [-175 175]*pi/180;
link(5).qlim = [-100 110]*pi/180; 
link(6).qlim = [-147.5 147.5]*pi/180; 

robot = SerialLink(link,'name','NiryoOne');
        
q = [0 -pi/2 0 0 0 0];                                                     % Create a vector of initial joint angles
        
robot.plot(q,'workspace',[-0.5 0.5 -0.5 0.5 -0.5 0.5],'scale',0.5);    
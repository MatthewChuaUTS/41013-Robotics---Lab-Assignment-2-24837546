% L1=Link('alpha',-pi/2,'a',0.103, 'd',0.70, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
% L2=Link('alpha',0,'a',0.80, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
% L3=Link('alpha',pi/2,'a',-0.210, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
% L4=Link('alpha',-pi/2,'a',0, 'd',0.30, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
% L5=Link('alpha',pi/2,'a',0.18, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
% L6=Link('alpha',0,'a',0.55, 'd',0.237, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);
% 
% densoRobot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Denso VM6083G');
% densoRobot.name = 'Denso VM6083G';
% % Use glyphs to draw robot, don't display the name
% densoRobot.plotopt = {'nojoints', 'noname', 'noshadow', 'nowrist'}; %
% 
% 
% densoRobot.plot(zeros(1,6)); % plot vector [0 0 0 0 0 0]



% L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% 
% robot = SerialLink([L1 L2 L3],'name','myRobot')                     % Generate the model
% 
% workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot        
% scale = 0.5;        
% q = zeros(1,3);                                                     % Create a vector of initial joint angles        
% robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

% Denso
% L1=Link('alpha',-pi/2,'a',0.180, 'd',0.475, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
% L2=Link('alpha',0,'a',0.385, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
% L3=Link('alpha',pi/2,'a',-0.100, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
% L4=Link('alpha',-pi/2,'a',0, 'd',0.329+0.116, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
% L5=Link('alpha',pi/2,'a',0, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
% L6=Link('alpha',0,'a',0, 'd',0.09, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);



% Niryo
% L1=Link('alpha',-pi/2,'a',0, 'd',0.08, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
% L2=Link('alpha',0,'a',0.210, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
% L3=Link('alpha',0,'a',-0.03, 'd',0.0415, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
% L4=Link('alpha',-pi/2,'a',0, 'd',0.30, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
% L5=Link('alpha',pi/2,'a',0.18, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
% L6=Link('alpha',0,'a',0.55, 'd',0.237, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);

% 
L1=Link('alpha',0,'a',0.08, 'd',0, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
L2=Link('alpha',0,'a',0.80, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
L3=Link('alpha',pi/2,'a',-0.210, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
L4=Link('alpha',-pi/2,'a',0, 'd',0.30, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
L5=Link('alpha',pi/2,'a',0.18, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
L6=Link('alpha',0,'a',0.55, 'd',0.237, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);

robot = SerialLink([L1 L2 L3 L4 L5 L6],'name','myRobot');
workspace = [-2 2 -2 2 0 2];                                       % Set the size of the workspace when drawing the robot        
scale = 0.5;        
q = zeros(1,6);                                                     % Create a vector of initial joint angles        
robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot



% L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
% 
% robot = SerialLink([L1 L2 L3],'name','myRobot')                     % Generate the model
% 
% workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot        
% scale = 0.5;        
% q = zeros(1,3);                                                     % Create a vector of initial joint angles        
% robot.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
% 


classdef niryoOne < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model on linear rails

    properties(Access = public)   
        plyFileNameStem = 'niryoOne'; % UR3e plyread
    end

    methods
%% Constructor
        function self = niryoOne(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);   % places the robot on specified coordinates
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end

            self.CreateModel();
			self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2); % rotx &y makes it stand up
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            self.model.teach([0,0,0,0,0,0,0]);


            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link from LinearUR5
            link(2) = Link([0      0.1519  0       pi/2    0]);
            link(3) = Link([0      0       -0.24365 0      0]);
            link(4) = Link([0      0       -0.21325 0      0]);
            link(5) = Link([0      0.11235 0       pi/2    0]);
            link(6) = Link([0      0.08535 0       -pi/2   0]);
            link(7) = Link([0      0.0819  0       0       0]);

            link(1).qlim = [-0.8 -0.01];
            link(2).qlim = [0 360]*pi/180;
            link(3).qlim = [-180 0]*pi/180;
            link(4).qlim = [0 270]*pi/180;
            link(5).qlim = [-180 180]*pi/180; %180 180
            link(6).qlim = [0 360]*pi/180; %-90 90
            link(7).qlim = [0 360]*pi/180; %-360 360

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end

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
			self.model.base = self.model.base.T * baseTr; % rotx &y makes it stand up
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            self.model.teach([0 -pi/2 0 0 0 0]);


            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link([0      0.103      0       -pi/2]);
            link(2) = Link([0      0          0.08    0]); 
            link(3) = Link([0      0          0.21    0]);
            link(4) = Link([0      -0.18      0       -pi/2]);
            link(5) = Link([0      0          0       pi/2]);
            link(6) = Link([0      0          0.0055       0]);
            
            link(1).qlim = [-175 175]*pi/180;
            link(2).qlim = [-90 36.7]*pi/180;
            link(3).qlim = [-80 90]*pi/180;
            link(4).qlim = [-175 175]*pi/180;
            link(5).qlim = [-100 110]*pi/180; 
            link(6).qlim = [-147.5 147.5]*pi/180; 

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end

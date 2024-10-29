classdef niryoOne < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model on linear rails

    properties(Access = public)   
        plyFileNameStem = 'n'; % UR3e plyread
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
			self.model.base = baseTr; % rotx &y makes it stand up * baseTr
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            % self.model.teach([0 0 0 0 0 0]); % I commented this out
            % because it rendered with the green and white checkerboard in
            % the main file. Feel free to uncomment it if you need to debug
            % and stuff. UR3


            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.183,'a',0,'alpha',pi/2,'qlim',deg2rad([-175 175]), 'offset',0); %
            link(2) = Link('d',0,'a',-0.21,'alpha',0,'qlim', deg2rad([-36.7 90]), 'offset',-pi/2); %
            link(3) = Link('d',0,'a',-0.03,'alpha',pi/2,'qlim', deg2rad([-90 80]), 'offset', 0);
            link(4) = Link('d',0.2215,'a',0,'alpha',pi/2,'qlim',deg2rad([-175 175]),'offset', 0);
            link(5) = Link('d',0,'a',0,'alpha',-pi/2,'qlim',deg2rad([-100 110]), 'offset',pi); % ignore offset for now
            link(6) = Link('d',0.00178,'a',0,'alpha',-pi,'qlim',deg2rad([-147.5 147.5]),'offset', 0);

            self.model = SerialLink(link,'name',self.name);
        end      
    end
end

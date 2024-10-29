classdef RobotPlate < handle
    %ROBOTPIGGY A class that creates a robot piggy
	%   The piggy can be moved around randomly. It is then possible to query
    %   the current location (base) of the piggy.    
    
    properties
        %> A single piggy model RobotCows
        plateModel;
    end
    
    methods
        %% Constructor
        function self = RobotPlate(baseTr)
            self.plateModel = self.getPlateModel('plate');
            self.plateModel.base = baseTr;
            % Plot 3D model
            % plot3d(self.piggyModel, 0, 'base', false);
            % self.PlotAndColourRobot();
            plot3d(self.plateModel, 0, 'workspace', [-1.6, 0.6, -1.6, 0.6, -0.7971, 1], 'view', [-30,30], 'noarrow', 'nowrist', 'color', {'hotpink'}, 'notiles');
            hold on;
        end
       
        %% Move the piggy forward and rotate randomly
        function movePlate(self, position)
            % Update piggy position
            self.plateModel.base = position;
            animate(self.plateModel, 0);                
            drawnow();
        end    
    end
    
    methods (Static)
        %% GetPlateModel
        function model = getPlateModel(name)
            if nargin < 1
                name = 'Plate';
            end
            
            [faceData, vertexData] = plyread('plate.ply', 'tri');
            link1 = Link('alpha', pi / 2, 'a', 0, 'd', 0.3, 'offset', 0);
            model = SerialLink(link1, 'name', name);
            model.faces = {[], faceData};
            model.points = {[], vertexData};
        end
    end    
end

classdef RobotPiggy < handle
    %ROBOTPIGGY A class that creates a robot piggy
	%   The piggy can be moved around randomly. It is then possible to query
    %   the current location (base) of the piggy.    
    
    properties
        %> A single piggy model RobotCows
        piggyModel;
    end
    
    methods
        %% Constructor
        function self = RobotPiggy(baseTr)
            self.piggyModel = self.getPiggyModel('piggy');
            self.piggyModel.base = baseTr;
            % Plot 3D model
            % plot3d(self.piggyModel, 0, 'base', false);
            % self.PlotAndColourRobot();
            plot3d(self.piggyModel, 0, 'workspace', [-1.6, 0.6, -1.6, 0.6, -0.7971, 1], 'view', [-30,30], 'noarrow', 'nowrist', 'color', {'hotpink'}, 'notiles');
            hold on;
        end
       
        %% Move the piggy forward and rotate randomly
        function movePiggy(self, position)
            % Update piggy position
            self.piggyModel.base = position;
            animate(self.piggyModel, 0);                
            drawnow();
        end    

        %% Check whether the pig intersects the light curtain
        function [intersectionPoint,check] = checkPigIntersection(self)
            % 0 if there is no intersection
            % 1 if there is a line plane intersection between the two points
            % 2 if the segment lies in the plane (always intersecting)
            % 3 if there is intersection point which lies outside line segment
        
            planeNormal = [0 1 0];  % Plane perpendicular to the y-axis.
            pointOnPlane = [0 -1 0]; % A point on the plane at y = -1.
            
            % % Define the object’s movement path (line segment)
            baseTr = self.piggyModel.base.T;
            baseTr = baseTr(1:3, 4)';

            point1OnLine = baseTr;
            point1OnLine = point1OnLine(2) - 0.25;
            point2OnLine = baseTr;  % End position of the pig.
            point2OnLine(2) = point2OnLine(2) + 0.25;
            
            [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine);
        end
    end
    
    methods (Static)
        %% GetPiggyModel
        function model = getPiggyModel(name)
            if nargin < 1
                name = 'Piggy';
            end
            
            [faceData, vertexData] = plyread('newPiggy.ply', 'tri');
            link1 = Link('alpha', pi / 2, 'a', 0, 'd', 0.3, 'offset', 0);
            model = SerialLink(link1, 'name', name);
            model.faces = {[], faceData};
            model.points = {[], vertexData};
        end
    end    
end

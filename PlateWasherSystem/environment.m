classdef environment < handle
    properties (Access = private)
        sponge;
        plate;
        warningLightHandle;
    end
    
    properties (Access = public)
        state = 0; % Make state public so it can be accessed and modified externally
    end
    
    properties (Constant, Access = private)
        WALL_HEIGHT = 1;
    end
    
    methods (Access = public)
        function self = environment()
            % Constructor: Sets up the environment and places objects
            self.setupEnvironment();
        end
        
        function updateWarningLight(self)
            % Public method to update the warning light
            self.setWarningLight();
        end
        
        function setState(self, newState)
            % Public method to set the state and update the warning light
            self.state = newState;
            self.updateWarningLight();
        end
    end

    methods (Access = private)
        function setWarningLight(self)
            try delete(self.warningLightHandle); end %#ok<TRYNC>
            if self.state == 0 
                self.warningLightHandle = PlaceObject('warningLightOff.ply', [1.5, -0.6, 0.6]);
            else
                self.warningLightHandle = PlaceObject('warningLightOn.ply', [1.5, -0.6, 0.6]);
            end
        end

        function setupEnvironment(self)
            clf;
            view(3)
            axis equal
            axis auto
            hold on 
            
            self.placeRoom();
            self.placeStaticEnvironmentObjects();
            self.sponge = PlaceObject('sponge.ply', [0.75, -0.06, 0]);
            self.setWarningLight();
        end

        function placeRoom(self)
            % Constants for room dimensions (local to this function)
            FLOOR_HEIGHT = -0.7971;
            ROOM_WIDTH = 1.5;

            % Places the floor and walls of the room
            % Place floor
            surf([0, 0; ROOM_WIDTH, ROOM_WIDTH], ...
                 [-1.6, 0.6; -1.6, 0.6], ...
                 [FLOOR_HEIGHT, FLOOR_HEIGHT; FLOOR_HEIGHT, FLOOR_HEIGHT], ...
                 'CData', imread('Floor.png'), ...
                 'FaceColor', 'texturemap');  

            % Place wall (back)
            surf([ROOM_WIDTH, ROOM_WIDTH; 0, 0], ...
                 [0.61, 0.61; 0.61, 0.61], ...
                 [FLOOR_HEIGHT, self.WALL_HEIGHT; FLOOR_HEIGHT, self.WALL_HEIGHT], ...
                 'CData', imread('wall.png'), ...
                 'FaceColor', 'texturemap');   

            % Place wall (right)
            surf([ROOM_WIDTH, ROOM_WIDTH; ROOM_WIDTH, ROOM_WIDTH], ...
                 [-1.6, -1.6; 0.61, 0.61], ...
                 [FLOOR_HEIGHT, self.WALL_HEIGHT; FLOOR_HEIGHT, self.WALL_HEIGHT], ...
                 'CData', imread('wall.png'), ...
                 'FaceColor', 'texturemap');  

            % Place Fence
            PlaceObject('fenceFinal.ply',[-0.05, -0.65, -0.7971]);
            PlaceObject('fenceFinal.ply',[-0.05, -0.35, -0.7971]);
            % verts = [get(h_1,'Vertices'), ones(size(get(h_1,'Vertices'),1),1)] * trotz(pi/2);
            % verts(:,1) = verts(:,1) * 20;
            % set(h_1,'Vertices',verts(:,1:3))
        end
        
        function placeStaticEnvironmentObjects(self)
            % Places static objects in the environment
            PlaceObject('stillEnvironment.ply', [0, 0, -0.0427]);
            PlaceObject('eStop.ply', [1.5, -0.6, 0.1]);     
            PlaceObject('fireExtinguisher.ply', [1.3, -0.1, -0.7971]);   
            PlaceObject('lightCurtain.ply', [0, -1, -0.7971]);
            PlaceObject('lightCurtain.ply', [1.45, -1, -0.7971]);
            self.plotLightCurtain();
        end

        function plotLightCurtain(~)
            bottomLeft = [0, -1, -0.7971];
            topRight = [1.45, -1, 0];
            laserCenters = 0.1; % laser distance
            laserArraySize = 1/laserCenters; % 10 lasers
            laserStartPoint = zeros(laserArraySize, 3);
            laserEndPoint = zeros(laserArraySize, 3);
            
            for i = 1:laserArraySize
                z = bottomLeft(3) + (i-1) * laserCenters; % add i distance from bottom 
                laserStartPoint(i, :) = [bottomLeft(1), bottomLeft(2), z];
                laserEndPoint(i, :) = [topRight(1), topRight(2), z];
            end
        
            for i = 1 : laserArraySize           
                hold on;
                plot3(laserStartPoint(i, 1),laserStartPoint(i, 2),laserStartPoint(i, 3) ,'r*');
                plot3(laserEndPoint(i, 1),laserEndPoint(i, 2),laserEndPoint(i, 3) ,'r*');
                plot3([laserStartPoint(i, 1),laserEndPoint(i, 1)],[laserStartPoint(i, 2),laserEndPoint(i, 2)],[laserStartPoint(i, 3),laserEndPoint(i, 3)] ,'r');
                axis equal
            end
        end
    end
end

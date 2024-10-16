classdef environment < handle
    properties (Access = private)
        sponge;
        plate;
        warningLightHandle;
        state = 0;
    end
    
    properties (Constant, Access = private)
        FLOOR_HEIGHT = -0.7971;
        ROOM_WIDTH = 1.5;
        ROOM_LENGTH = 2.2; % -1.6 to 0.6
        WALL_HEIGHT = 1;
    end
    
    methods (Access = public)
        function self = environment()
            % Constructor: Sets up the environment and places objects
            self.setupEnvironment();
            self.sponge = self.placeObject('sponge.ply', [0.75, -0.06, 0]);
            self.setWarningLight();
        end
    end

    methods (Access = private)
        function setWarningLight(self)
            % Sets the warning light based on the current state
            try delete(self.warningLightHandle); end
            if self.state == 0
                self.warningLightHandle = self.placeObject('warningLightOff.ply', [1.5, -0.6, 0.6]);
            else
                self.warningLightHandle = self.placeObject('warningLightOn.ply', [1.5, -0.6, 0.6]);
            end
        end

        function setupEnvironment(self)
            % Sets up the basic environment including view and axes
            clf;
            view(3)
            axis equal
            axis auto
            hold on 
            
            self.placeRoom();
            self.placeStaticEnvironmentObjects();
        end

        function placeRoom(self)
            % Places the floor and walls of the room
            % Place floor
            surf([0, 0; self.ROOM_WIDTH, self.ROOM_WIDTH], ...
                 [-1.6, 0.6; -1.6, 0.6], ...
                 [self.FLOOR_HEIGHT, self.FLOOR_HEIGHT; self.FLOOR_HEIGHT, self.FLOOR_HEIGHT], ...
                 'CData', imread('Floor.png'), ...
                 'FaceColor', 'texturemap');  

            % Place wall (back)
            surf([self.ROOM_WIDTH, self.ROOM_WIDTH; 0, 0], ...
                 [0.61, 0.61; 0.61, 0.61], ...
                 [self.FLOOR_HEIGHT, self.WALL_HEIGHT; self.FLOOR_HEIGHT, self.WALL_HEIGHT], ...
                 'CData', imread('wall.png'), ...
                 'FaceColor', 'texturemap');   

            % Place wall (right)
            surf([self.ROOM_WIDTH, self.ROOM_WIDTH; self.ROOM_WIDTH, self.ROOM_WIDTH], ...
                 [-1.6, -1.6; 0.61, 0.61], ...
                 [self.FLOOR_HEIGHT, self.WALL_HEIGHT; self.FLOOR_HEIGHT,  self.WALL_HEIGHT], ...
                 'CData', imread('wall.png'), ...
                 'FaceColor', 'texturemap');  
        end
        
        function placeStaticEnvironmentObjects(self)
            % Places static objects in the environment
            self.placeObject('stillEnvironment.ply', [0, 0, -0.0427]);
            self.placeObject('eStop.ply', [1.5, -0.6, 0.1]);     
            self.placeObject('fireExtinguisher.ply', [1.3, -0.1, self.FLOOR_HEIGHT]);   
            self.placeObject('lightCurtain.ply', [0, -1, self.FLOOR_HEIGHT]);
            self.placeObject('lightCurtain.ply', [1.45, -1, self.FLOOR_HEIGHT]);
            self.placeObject('piggy.ply', [0.8, -1.3, self.FLOOR_HEIGHT]);    
            self.plotLightCurtain();
        end

        function handle = placeObject(~, filename, position)
            handle = PlaceObject(filename, position);
        end

        function plotLightCurtain(~)
            bottomLeft = [0, -1, -0.7971];
            topRight = [1.45, -1, 0];
            laserCenters = 0.1;
            laserArraySize = 1/laserCenters;
            laserStartPoint = zeros(laserArraySize, 3);
            laserEndPoint = zeros(laserArraySize, 3);
            
            for i = 1:laserArraySize
                z = bottomLeft(3) + (i-1) * laserCenters;
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



% classdef environment < handle
%     properties
%         sponge;
%         plate;
%         warningLightHandle;
%         state = 0;
%     end
% 
%     properties (Constant)
%         FLOOR_HEIGHT = -0.7971;
%         ROOM_WIDTH = 1.5;
%         ROOM_LENGTH = 2.2; % -1.6 to 0.6
%         WALL_HEIGHT = 1;
%     end
% 
%     methods
%         function obj = environment()
%             % Constructor: Sets up the environment and places objects
%             obj.setupEnvironment();
%             obj.sponge = PlaceObject('sponge.ply', [0.75, -0.06, 0]);
%             obj.setWarningLight();
%         end
% 
%         function setWarningLight(obj)
%             % Sets the warning light based on the current state
%             try delete(obj.warningLightHandle); end
%             if obj.state == 0
%                 obj.warningLightHandle = PlaceObject('warningLightOff.ply', [1.5, -0.6, 0.6]);
%             else
%                 obj.warningLightHandle = PlaceObject('warningLightOn.ply', [1.5, -0.6, 0.6]);
%             end
%         end
% 
%         function setupEnvironment(obj)
%             % Sets up the basic environment including view and axes
%             clf;
%             view(3)
%             axis equal
%             axis auto
%             hold on 
% 
%             obj.placeRoom();
%             obj.placeStaticEnvironmentObjects();
%         end
% 
%         function placeRoom(obj)
%             % Places the floor and walls of the room
%             % Place floor
%             surf([0, 0; obj.ROOM_WIDTH, obj.ROOM_WIDTH], ...
%                  [-1.6, 0.6; -1.6, 0.6], ...
%                  [obj.FLOOR_HEIGHT, obj.FLOOR_HEIGHT; obj.FLOOR_HEIGHT, obj.FLOOR_HEIGHT], ...
%                  'CData', imread('Floor.png'), ...
%                  'FaceColor', 'texturemap');  
% 
%             % Place wall (back)
%             surf([obj.ROOM_WIDTH, obj.ROOM_WIDTH; 0, 0], ...
%                  [0.61, 0.61; 0.61, 0.61], ...
%                  [obj.FLOOR_HEIGHT, obj.WALL_HEIGHT; obj.FLOOR_HEIGHT, obj.WALL_HEIGHT], ...
%                  'CData', imread('wall.png'), ...
%                  'FaceColor', 'texturemap');   
% 
%             % Place wall (right)
%             surf([obj.ROOM_WIDTH, obj.ROOM_WIDTH; obj.ROOM_WIDTH, obj.ROOM_WIDTH], ...
%                  [-1.6, -1.6; 0.61, 0.61], ...
%                  [obj.FLOOR_HEIGHT, obj.WALL_HEIGHT; obj.FLOOR_HEIGHT,  obj.WALL_HEIGHT], ...
%                  'CData', imread('wall.png'), ...
%                  'FaceColor', 'texturemap');  
%         end
% 
%         function placeStaticEnvironmentObjects(obj)
%             % Places static objects in the environment
%             PlaceObject('stillEnvironment.ply', [0, 0, -0.0427]);
%             PlaceObject('eStop.ply', [1.5, -0.6, 0.1]);     
%             PlaceObject('fireExtinguisher.ply', [1.3, -0.1, obj.FLOOR_HEIGHT]);   
%             PlaceObject('lightCurtain.ply', [0, -1, obj.FLOOR_HEIGHT]);
%             PlaceObject('lightCurtain.ply', [1.45, -1, obj.FLOOR_HEIGHT]);
%             PlaceObject('piggy.ply', [0.8, -1.3, obj.FLOOR_HEIGHT]);    
%             plotLightCurtain();
%         end
%     end
%     methods (Access = private)
%         function plotLightCurtain()
%             bottomLeft = [0, -1, -0.7971];
%             topRight = [1.45, -1, 0];
%             laserCenters = 0.1;
%             laserArraySize = 1/laserCenters;
%             laserStartPoint = zeros(laserArraySize, 3);
%             laserEndPoint = zeros(laserArraySize, 3);
% 
%             for i = 1:laserArraySize
%                 z = bottomLeft(3) + (i-1) * laserCenters;
%                 laserStartPoint(i, :) = [bottomLeft(1), bottomLeft(2), z];
%                 laserEndPoint(i, :) = [topRight(1), topRight(2), z];
%             end
% 
%             for i = 1 : laserArraySize           
%                 hold on;
%                 plot3(laserStartPoint(i, 1),laserStartPoint(i, 2),laserStartPoint(i, 3) ,'r*');
%                 plot3(laserEndPoint(i, 1),laserEndPoint(i, 2),laserEndPoint(i, 3) ,'r*');
%                 plot3([laserStartPoint(i, 1),laserEndPoint(i, 1)],[laserStartPoint(i, 2),laserEndPoint(i, 2)],[laserStartPoint(i, 3),laserEndPoint(i, 3)] ,'r');
%                 axis equal
% 
%             end
%         end
%     end
% end


% classdef environment < handle
%     properties
%         sponge;
%         plate;
%         warningLightHandle;
%         state = 0;
%     end
% 
%     methods
%         function obj = environment()
%             % Constructor
%             obj.setupEnvironment();
%             obj.sponge = PlaceObject('sponge.ply', [0.75, -0.06, 0]);
%             obj.plate = PlaceObject('plate.ply', [0.5, 0, 0]);
%             setWarningLight(obj);
%         end
% 
%         function setWarningLight(obj)
%             try delete(obj.warningLightHandle); end
%             if obj.state == 0
%                 obj.warningLightHandle = PlaceObject('warningLightOff.ply',[1.5,-0.6,0.6]);
%             else
%                 obj.warningLightHandle = PlaceObject('warningLightOn.ply',[1.5,-0.6,0.6]);
%             end
%         end
% 
%         function setupEnvironment(obj)
%             clf;
%             view(3)
%             axis equal
%             axis auto
%             hold on 
% 
%             obj.placeRoom();
%             obj.placeStaticEnvironmentObjects();
%         end
% 
%         function placeRoom(obj)
%             % Place floor
%             surf([ 0,0; 1.5,1.5] ...
%                 ,[-1.6,0.6;-1.6,0.6] ...
%                 ,[-0.7971,-0.7971;-0.7971,-0.7971] ... %height of floor
%                 ,'CData',imread('Floor.png') ...
%                 ,'FaceColor','texturemap');  
% 
%             % Place wall (back)
%             surf([1.5, 1.5;0,0] ...
%                 ,[0.61,0.61;0.61,0.61] ...
%                 ,[-0.7971,1;-0.7971,1] ... %height of floor
%                 ,'CData',imread('wall.png') ...
%                 ,'FaceColor','texturemap');   
% 
%             % Place wall (right)
%             surf([1.5, 1.5;1.5, 1.5] ...
%                 ,[-1.6,-1.6;0.61,0.61] ...
%                 ,[-0.7971,1;-0.7971,1] ... %height of floor
%                 ,'CData',imread('wall.png') ...
%                 ,'FaceColor','texturemap');  
%         end
% 
%         function placeStaticEnvironmentObjects(obj)
%             % Place plyFiles
%             PlaceObject('stillEnvironment.ply',[0,0,-0.0427]); % z=0 is where the robots stand
%             PlaceObject('eStop.ply',[1.5,-0.6,0.1]);     
%             PlaceObject('fireExtinguisher.ply', [1.3, -0.1, -0.7971]);   
%             PlaceObject('lightCurtain.ply', [0, -1, -0.7971]);
%             PlaceObject('lightCurtain.ply', [1.45, -1, -0.7971]);
%             PlaceObject('piggy.ply', [0.8, -1.3, -0.7971]);    
%             % PlaceObject('warningLightOff.ply',[1.5,-0.6,0.6]);
%             plotLightCurtain();
%         end
%     end
% end
%             % 
%             % try delete(obj.warningLightHandle); end
%             % if state == 0
%             %     obj.warningLightHandle = PlaceObject('warningLightOff.ply',[1.5,-0.6,0.6]);
%             % else
%             %     obj.warningLightHandle = PlaceObject('warningLightOn.ply',[1.5,-0.6,0.6]);
%             % end
%             % obj.warningLight = state;
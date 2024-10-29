classdef RobotPlate < handle
    % PLATE A class that creates a brick object using the plate.ply file
    
    properties
        plateMesh_h;
        platePose;
        plateVerts;
        platevCount;
    end
    
    methods
        %% Constructor
        function self = RobotPlate(position)
            if nargin < 1
                position = eye(4);  % Set custom position if provided
            end
            self.platePose = position;
            [f,v,data] = plyread('plate.ply','tri');
            self.platevCount = size(v,1);                                                    % Get vertex count
            midPoint = sum(v)/self.platevCount;                                              % Move center point to origin
            self.plateVerts = v - repmat(midPoint,self.platevCount,1);
            
            %platePose = transl(-0.2,-0.5,0.1);
            % self.platePose = position;
            plateVerts_hom = [self.plateVerts, ones(self.platevCount, 1)];  % 964x4                  % Convert brickVerts to homogeneous coordinates (964x4)
            updatedPoints = (self.platePose * plateVerts_hom')';                             % Transform the vertices
            self.plateMesh_h = trisurf(f, updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',[data.vertex.red, data.vertex.green, data.vertex.blue] / 255, ...
                'EdgeColor','interp','EdgeLighting','flat');                            % Now update the mesh Vertices
            drawnow(); 
        end
        %% Plot
        function PlotPlate(self,newPosition)
            self.platePose = newPosition;
            plateVerts_hom = [self.plateVerts, ones(self.platevCount, 1)]; 
            updatedPoints = (self.platePose * plateVerts_hom')';
            self.plateMesh_h.Vertices = updatedPoints(:,1:3);
            drawnow();
        end
       
        function pose = get_plate_pose(self)
            pose = self.platePose;
        end
        %% Delete
        function Hide(self)
            set(self.plateMesh_h, 'Visible', 'off');
        end
    end
end

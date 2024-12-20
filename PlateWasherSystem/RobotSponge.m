classdef RobotSponge < handle
    % PLATE A class that creates a brick object using the plate.ply file
    
    properties
        spongeMesh_h;
        spongePose;
        spongeVerts;
        spongevCount;
    end
    
    methods
        %% Constructor
        function self = RobotSponge(position)
            if nargin < 1
                position = eye(4);
            end
            self.spongePose = position;
            [f,v,data] = plyread('sponge.ply','tri');
            self.spongevCount = size(v,1);                                                    % Get vertex count
            midPoint = sum(v)/self.spongevCount;                                              % Move center point to origin
            self.spongeVerts = v - repmat(midPoint,self.spongevCount,1);
            
            %spongePose = transl(-0.2,-0.5,0.1);
            self.spongePose = position;
            spongeVerts_hom = [self.spongeVerts, ones(self.spongevCount, 1)];  % 964x4                  % Convert brickVerts to homogeneous coordinates (964x4)
            updatedPoints = (self.spongePose * spongeVerts_hom')';                             % Transform the vertices
            self.spongeMesh_h = trisurf(f, updatedPoints(:,1), updatedPoints(:,2), updatedPoints(:,3) ...
                ,'FaceVertexCData',[data.vertex.red, data.vertex.green, data.vertex.blue] / 255, ...
                'EdgeColor','interp','EdgeLighting','flat');                            % Now update the mesh Vertices
            drawnow(); 
        end
        %% Plot
        function PlotSponge(self,newPosition)
            self.spongePose = newPosition;
            spongeVerts_hom = [self.spongeVerts, ones(self.spongevCount, 1)]; 
            updatedPoints = (self.spongePose * spongeVerts_hom')';
            self.spongeMesh_h.Vertices = updatedPoints(:,1:3);
            drawnow();
        end
       
        function pose = get_sponge_pose(self)
            pose = self.spongePose;
        end
        %% Delete 
        function Hide(self)
            set(self.spongeMesh_h, 'Visible', 'off');
        end
    end
end

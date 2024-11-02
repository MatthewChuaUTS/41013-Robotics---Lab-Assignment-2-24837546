%% LinePlaneIntersection
% This function calculates the intersection point between a line segment
% and a plane. It checks various conditions to determine the nature of
% the intersection.

% Inputs:
% - planeNormal: A vector normal to the plane.
% - pointOnPlane: A point on the plane.
% - point1OnLine: The first point defining the line segment.
% - point2OnLine: The second point defining the line segment.

% Outputs:
% - intersectionPoint: The point of intersection (if any) between the line
%   segment and the plane.
% - check: A flag indicating the type of intersection:
%   0 - No intersection.
%   1 - Intersection between the two points of the segment.
%   2 - The entire segment lies in the plane.
%   3 - Intersection exists but lies outside the line segment.

function [intersectionPoint, check] = LinePlaneIntersection(planeNormal, pointOnPlane, point1OnLine, point2OnLine)
    
    % Initialize the intersection point to origin (default value)
    intersectionPoint = [0 0 0];
    
    % Direction vector of the line segment
    u = point2OnLine - point1OnLine;
    
    % Vector from a point on the line to a point on the plane
    w = point1OnLine - pointOnPlane;
    
    % Dot product of the plane normal and the line direction vector
    D = dot(planeNormal, u);
    
    % Negative dot product of the plane normal and vector w
    N = -dot(planeNormal, w);
    
    % Initialize check value (default: no intersection)
    check = 0; %#ok<NASGU>
    
    % Check if the line segment is parallel to the plane
    if abs(D) < 10^-7
        % Check if the segment lies in the plane
        if N == 0
            check = 2; % The segment lies in the plane
            return
        else
            check = 0; % The segment is parallel and not in the plane
            return
        end
    end
    
    % Compute the intersection parameter (sI)
    sI = N / D;
    
    % Calculate the intersection point
    intersectionPoint = point1OnLine + sI .* u;
    
    % Determine the type of intersection
    if (sI < 0 || sI > 1)
        check = 3; % Intersection point lies outside the segment
    else
        check = 1; % Intersection point lies within the segment
    end
end

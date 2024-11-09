% Initialize qWaypoints as [q1, q2]  // Define waypoints for the trajectory
% Set isCollision to true              // Initialize collision flag
% Set checkedTillWaypoint to 1         // Track the last checked waypoint
% Initialize qMatrix as empty          // Matrix to store the trajectory
% 
% While isCollision do                  // Continue until a valid trajectory is found
%     Set startWaypoint to checkedTillWaypoint  // Starting waypoint for this iteration
% 
%     For i from startWaypoint to size(qWaypoints) - 1 do  // Loop through waypoints
%         Set qMatrixJoin to InterpolateWaypointRadians(qWaypoints[i, i+1], deg2rad(10))  // Interpolate
% 
%         If not IsCollision(robot, qMatrixJoin, faces, vertex, faceNormals) then  // Check for collisions
%             Append qMatrixJoin to qMatrix  // Add valid segment to trajectory matrix
%             Animate robot using qMatrixJoin  // Animate the robot along the segment
% 
%             Set isCollision to false  // No collision detected
%             Set checkedTillWaypoint to i + 1  // Update last checked waypoint
% 
%             // Attempt to connect to the final goal (q2)
%             Set qMatrixJoin to InterpolateWaypointRadians([last element of qMatrix, q2], deg2rad(10)) 
% 
%             If not IsCollision(robot, qMatrixJoin, faces, vertex, faceNormals) then  // Check if connection is valid
%                 Append qMatrixJoin to qMatrix  // Add goal connection to trajectory
%                 Break  // Exit loop as valid path is found
%             End If
%         Else  
%             // Collision detected, generate random joint configuration
%             Set qRand to (2 * random(1, 3) - 1) * pi  // Generate random configuration
% 
%             While IsCollision(robot, qRand, faces, vertex, faceNormals) do  // Regenerate until no collision
%                 Set qRand to (2 * random(1, 3) - 1) * pi  // Continue generating
%             End While  
% 
%             // Insert new random pose into waypoints
%             Set qWaypoints to [qWaypoints[1:i], qRand, qWaypoints[i+1:end]]  
%             Set isCollision to true  // Reset collision flag for next iteration
%             Break  // Exit loop to recheck from updated waypoints
%         End If  
%     End For  
% End While
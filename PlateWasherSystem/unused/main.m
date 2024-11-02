clf;
clear all;

niryoWaypoints = {transl([0.22,0.061,0.195]) ... % plate 1
    , transl([0.358,0.196,0.244]) ... % point2 2
    , transl([0.358,0.196,-0.005]) ... % dunk 3
    , transl([0.358,0.196,0.406]) ... % point4 4  RMRC in lab happens for same axis, goes straight, is it kay if the points are short?
    , transl([0.6975,0.263,0.3915]) ... % hold 5
    , transl([0.6975,0.263,0.3915]) ... % hold 6
    , transl([0.6975,0.263,0.3915]) ... % hold 7
    , transl([0.6975,0.263,0.3915]) ... % hold 8
    , transl([0.6975,0.263,0.3915]) ... % hold 9
    , transl([0.747,-0.049,0.072]) ... % sponge 10
    , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % scrub position 11
    , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % scrub 12
    , transl([0.726,0.1495,0.4695]) * trotx(pi/2) ... % srub 13
    , transl([0.726,0.1495,0.4695]) ... % scrub position 14
    , transl([0.747,-0.049,0.072]) ... % sponge 15
    , transl([0.3203,-0.01,0.423]) * trotx(-pi/2) ... % reset 16
    , transl([0.3203,-0.01,0.423]) * trotx(-pi/2) ... % reset 17
    , transl([0.3203,-0.01,0.423]) * trotx(-pi/2)}; % reset 18

Ur3Waypoints = {transl([0.747,-0.049,0.072]) * troty(pi) ... % sponge 1
    , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 2
    , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 3
    , transl([0.7795, 0.1269, 0.3629]) * troty(pi) ...  % srub position 4
    , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub position 5
    , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub 6
    , transl([0.7795, 0.1269, 0.3629]) * troty(pi) * trotx(-pi/2) ...  % srub 7
    , transl([0.747,-0.049,0.072]) * troty(pi) ... % sponge 8
    , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 9
    , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 10
    , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 11
    , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 12
    , transl([0.8247, 0.2134, 0.5401]) * troty(pi) ...  % hold 13
    , transl([0.6979, 0.2587, -0.04671]) * troty(pi) ...   % dunk 14
    , transl([0.6979, 0.2587, 0.5401]) * troty(pi) ...   % up 15
    , transl([1.165, 0.3, 0.178]) * troty(pi) ... % place 16
    , transl([0.9346, -0.1223, 0.5269]) * troty(pi) ...  % reset 17
    , transl([0.9346, -0.1223, 0.5269]) * troty(pi)};  % reset 18

env = environment();

showCollision = 1;
if showCollision
    centerpnt = [0.15, 0.04, 0.21];
    side = 0.2;
    plotOptions.plotFaces = true; %change for render during demo?
    [vertex, faces, faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2, plotOptions);
else 
    centerpnt = [0, 0.61, 1];
    side = 0.00001;
    plotOptions.plotFaces = true;
    [vertex, faces, faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2, plotOptions);
end


% Set up Robot 1
niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];  
myNiryoOne = niryoOne(transl(0.54, -0.01, 0));  
myNiryoOne.model.animate(niryoOneCurrentJointPosition);  

% Set up Robot 2
% NOTE, SET UR3 QLIM JOINT 2 TO 30 AND 150 DEG, WHATEVER THAT IS IN RAD
UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];  
myUR3 = UR3e(transl(1.02, -0.01, 0));                                              
myUR3.model.animate(UR3CurrentJointPosition); 

% Set up plate
initialPlatePose = transl(0.22, 0.061, 0.195); 
plate = RobotPlate(initialPlatePose);


% Set up sponge
initialSpongePose = transl(0.747,-0.049,0.072); 
sponge = RobotSponge(initialSpongePose);


% Animatestep
steps = 50;
[niryoTrajectoryQmatrix, collisionFlagN] = calculateRobotTrajectory(myNiryoOne, niryoWaypoints, steps, vertex, faces, faceNormals);
[ur3TrajectoryQmatrix, collisionFlagU] = calculateRobotTrajectory(myUR3, Ur3Waypoints, steps, vertex, faces, faceNormals);

collisionDetected = collisionFlagN || collisionFlagU;
safety = 1;

for waypoint = 1:size(niryoTrajectoryQmatrix, 2)
    for jointStep = 1:size(niryoTrajectoryQmatrix{1}, 1)
        if safety == 1
            myNiryoOne.model.animate(niryoTrajectoryQmatrix{waypoint}(jointStep,:));
            myUR3.model.animate(ur3TrajectoryQmatrix{waypoint}(jointStep,:));
            niryoEndEffectorPose = myNiryoOne.model.fkine(niryoTrajectoryQmatrix{waypoint}(jointStep,:));


             if (waypoint <= 8)
                % Mount plate
                plateAttached = true;
                platePose = niryoEndEffectorPose.T * transl(0, 0, -0.05) * trotx(pi/2);
                plate.PlotPlate(platePose);
            elseif (waypoint == 9 && plateAttached)
                % Detach plate
                plateAttached = false;
                plate.Hide();            
            end

            if (waypoint >= 10 && waypoint <= 15)
                % Mount sponge
                spongeAttached = true;
                spongePose = niryoEndEffectorPose.T * transl(0, 0, -0.05);
                sponge.PlotSponge(spongePose);
            elseif (waypoint == 16 && spongeAttached)
                % Detach sponge
                spongeAttached = false;
                sponge.Hide();
            end

            drawnow();
            pause(0.01);
        else
            outputBar.Value = 'Sequence paused due to safety concerns';
            return;
        end
    end
    if collisionDetected
        break;
    end
end
% 
% if collisionDetected
%     outputBar.Value = 'Collision detected! Animation stopped.';
% else
%     outputBar.Value = 'Sequence completed successfully';
% end











%% bury
% steps = 50;
% niryoTrajectoryQmatrix = calculateRobotTrajectory(myNiryoOne, niryoWaypoints, steps);
% ur3TrajectoryQmatrix = calculateRobotTrajectory(myUR3, Ur3Waypoints, steps);
% 
% for waypoint = 1:size(niryoTrajectoryQmatrix, 2)
%     for jointStep=1:size(niryoTrajectoryQmatrix{1}, 1)
%          myNiryoOne.model.animate(niryoTrajectoryQmatrix{waypoint}(jointStep,:));
%          myUR3.model.animate(ur3TrajectoryQmatrix{waypoint}(jointStep,:));
%          pause(0.01);
%     end
% end  
% 
% UR3CurrentJointPosition = [0, -pi/2, 0, 0, -pi/2, 0];  
% app.myUR3.model.animate(UR3CurrentJointPosition);

% for i=1:size(niryoWaypoints, 2)
%     myNiryoOne.model.animate(myNiryoOne.model.ikcon(niryoWaypoints{i}));
%     disp(i);
%     input("press enter to continue");
% end


% for i=1:size(Ur3Waypoints, 2)
%     myUR3.model.animate(myUR3.model.ikcon(Ur3Waypoints{i}));
%     disp(i);
%     input("press enter to continue");
% end
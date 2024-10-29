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

% env = environment();

% Set up Robot 1
% niryoOneCurrentJointPosition = [0, 0, 0, 0, 0, 0];  
% myNiryoOne = niryoOne(transl(0.54, -0.01, 0));  
% myNiryoOne.model.animate(niryoOneCurrentJointPosition);  

% Set up plate
initialPlatePose = transl(0.22, 0.061, 0.195); 
plate = RobotPlate(initialPlatePose);

% Set up sponge
initialSpongePose = transl(0.747,-0.049,0.072); 
sponge = RobotSponge(initialSpongePose);

% Animatestep
steps = 50;
niryoTrajectoryQmatrix = calculateRobotTrajectory(myNiryoOne, niryoWaypoints, steps);

% qmatrix = zeros((length(niryoWaypoints) - 1) * steps, 6);
qWaypoints = zeros((length(niryoWaypoints)) * steps, 6);  
deltaT = 0.05;   
    
    % Generate the joint angle using ikcon
    for i = 1:length(niryoWaypoints)
        qWaypoints(i, :) = myUR3.model.ikcon(niryoWaypoints{i}, niryoOneCurrentJointPosition);
        niryoOneCurrentJointPosition = qWaypoints(i, :); 
    end

    rowIdx = 1; 
    plateAttached = false;
    spongeAttached = false;

    for waypoint = 1:size(niryoTrajectoryQmatrix, 2)
        for jointStep=1:size(niryoTrajectoryQmatrix{1}, 1)
            niryoEndEffectorPose = myNiryoOne.model.fkine(niryoTrajectoryQmatrix{waypoint}(jointStep,:)).T;
            if (waypoint <= 8)
                % Mount plate
                plateAttached = true;
                platePose = niryoEndEffectorPose * transl(0, 0, -0.05) * trotx(pi/2);
                plate.PlotPlate(platePose);
            elseif (waypoint == 9 && plateAttached)
                % Detach plate
                plateAttached = false;
                plate.Hide();            
            end

            if (waypoint >= 10 && waypoint <= 15)
                % Mount sponge
                spongeAttached = true;
                spongePose = niryoEndEffectorPose * transl(0, 0, -0.05);
                sponge.PlotSponge(spongePose);
            elseif (waypoint == 16 && spongeAttached)
                % Detach sponge
                spongeAttached = false;
                sponge.Hide();
            end
        end
        rowIdx = rowIdx + steps;
    end

    % niryoTrajectoryQmatrix = qmatrix;

    % myNiryoOne.model.delay = 0;
    % for j = 1:size(niryoTrajectoryQmatrix, 1)
    %     myNiryoOne.model.animate(niryoTrajectoryQmatrix(j, :));
    %     drawnow(); 
    %     pause(0.01);  
    % end



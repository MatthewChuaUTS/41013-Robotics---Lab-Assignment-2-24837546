function clean(myNiryoOne, myUR3)
    steps = 50;
    stages = 12;
    totalSteps = stages*steps;
    
    % Calculate Trajectories
    niryoTrajectory = calculateNiryoTrajectory(totalSteps);
    ur3Trajectory = calculateUr3Trajectory(totalSteps);

    for currentStep=1:totalSteps
        safetyFlag = safety();
        if safetyFlag == 0
            myNiryoOne.model.animate(niryoTrajectory(currentStep,:));  
            myUR3.model.animate(ur3Trajectory(currentStep,:)); 
        else
            disp('Emergency stop activated! Cleaning process paused.');
            currentStep = currentStep - 1;
        end
    end
end

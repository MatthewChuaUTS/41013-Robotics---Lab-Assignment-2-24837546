function clean(myNiryoOne, myUR3)
    steps = 50;
    stages = 12;
    totalSteps = stages*steps;
    resumeFlag = 1;
    UR3
    % Calculate Trajectories
    niryoTrajectory = calculateNiryoTrajectory(totalSteps);
    ur3Trajectory = calculateUr3Trajectory(totalSteps);

    currentStep = 1;
    while currentStep <= totalSteps % <= is correct
        safetyFlag = safety();
        if safetyFlag == 0 && resumeFlag == 1
            myNiryoOne.model.animate(niryoTrajectory(currentStep,:));  
            myUR3.model.animate(ur3Trajectory(currentStep,:));
        else
            disp('Emergency stop activated! Cleaning process paused.');
            currentStep = currentStep - 1;
            disarm = checkDisarm(); % need some sort of toggled button
            resumeFlag = checkResume(disarm);
        end
        currentStep = currentStep + 1;
    end
end

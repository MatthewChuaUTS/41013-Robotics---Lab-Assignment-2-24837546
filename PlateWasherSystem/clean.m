function clean(myNiryoOne, myUR3)
    steps = 50;
    stages = 12;
    totalSteps = stages*steps;
    resumeFlag = 1;

    % Calculate Trajectories
    niryoTrajectoryQmatrix = calculateNiryoTrajectory(myNiryoOne, totalSteps);
    ur3TrajectoryQmatrix = calculateUr3Trajectory(myUR3, totalSteps);

    currentStep = 1;
    while currentStep <= totalSteps % <= is correct
        safetyFlag = safety();
        if safetyFlag == 0 && resumeFlag == 1
            myNiryoOne.model.animate(niryoTrajectoryQmatrix(currentStep,:));  
            myUR3.model.animate(ur3TrajectoryQmatrix(currentStep,:));
        else
            disp('Emergency stop activated! Cleaning process paused.');
            currentStep = currentStep - 1;
            disarm = checkDisarm();
            resumeFlag = checkResume(disarm);
        end
        currentStep = currentStep + 1;
    end
end

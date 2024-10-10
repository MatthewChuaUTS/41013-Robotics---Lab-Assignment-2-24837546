function clean ()
    steps = 12; % based on the cleanSteps diagram
    animateAtOnce = false; % can be used to either calculate everything then animate, or each step and animate intermittently
    for currentStep=1:steps
        safety();
        cleanSteps(currentStep);
    end
end
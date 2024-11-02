function safetyFlag = safety()
    % will need to interact with env as well
    eStopFlag = eStop(); % Check both e-stop buttons
    barrierFlag = barrierDisruption(); % Check barrier and pig
    
    % Determine safety status
    if eStopFlag || barrierFlag
        safetyFlag = 1; % Indicate unsafe condition
    else
        safetyFlag = 0; % Indicate safe condition
    end
end

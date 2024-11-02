%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function niryoTrajectoryQmatrix = FineInterpolation(q1,q2,maxStepRadians)
    if nargin < 3
        maxStepRadians = deg2rad(1);
    end

    steps = 2;
    while true
        traj = jtraj(q1,q2,steps);
        
        if all(abs(diff(traj)) < maxStepRadians)
            break;
        end
        steps = steps + 1;
    end

    niryoTrajectoryQmatrix = jtraj(q1,q2,steps);
end

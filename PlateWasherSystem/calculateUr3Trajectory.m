function [combinedQmatrix] = calculateUr3Trajectory(myUr3, spongePosition)
        % spongePosition = [0.748, -0.06, 0.05]
        % Calculate movement to the brick from last position
        myUR3.model.fkine(currentJointPosition)

        qTarget = spongePosition * transl(0, 0, 0.3) * trotx(pi) * trotz(pi/2);
        finalJoints = myUR3.model.ikcon(qTarget, currentJointPosition);
        combinedQmatrix = [combinedQmatrix; jtraj(currentJointPosition, finalJoints, steps)];
        currentJointPosition = finalJoints;

end
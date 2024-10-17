classdef testGui < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        % [Your existing UI component properties]
        % ... (omitted for brevity)
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
    end

    % Properties for robot instances and other variables
    properties (Access = public)
        myNiryoOne;
        myUR3;
        piggy;
        env;

        % Joint variables for Niryo One
        myNiryoOneLink1;
        myNiryoOneLink2;
        myNiryoOneLink3;
        myNiryoOneLink4;
        myNiryoOneLink5;
        myNiryoOneLink6;

        % Joint variables for UR3
        myUR3Link1;
        myUR3Link2;
        myUR3Link3;
        myUR3Link4;
        myUR3Link5;
        myUR3Link6;

        eStopFlag = -1; % 1 is pressed
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Remove redundant initializations
            app.outputBar.Value = "GUI loaded.";
            % Additional GUI-related initialization can be added here
        end

        % Value changing function: JOINT1Knob
        function JOINT1KnobValueChanging(app, event)
            changingJointValue = event.Value;
            app.myUR3Link1 = deg2rad(changingJointValue);
            app.myUR3.model.animate([app.myUR3Link1, app.myUR3Link2, app.myUR3Link3, app.myUR3Link4, app.myUR3Link5, app.myUR3Link6]);
        end

        % [Similarly update other callback functions for JOINT2Knob, JOINT3Knob, etc.]

        % Value changing function: PigMoverSlider
        function PigMoverSliderValueChanging(app, event)
            changingValue = event.Value;
            app.piggy.movePiggy(transl(0.8, changingValue, -0.7971));
        end

        % Button pushed function: MoveNiryoButton
        function MoveNiryoButtonPushed(app, event)
            xPos = app.XPosEditField.Value;
            yPos = app.YPosEditField.Value;
            zPos = app.ZPosEditField.Value;
            targetPose = transl(xPos, yPos, zPos);

            currentJointPositions = [app.myNiryoOneLink1, app.myNiryoOneLink2, app.myNiryoOneLink3, app.myNiryoOneLink4, app.myNiryoOneLink5, app.myNiryoOneLink6];
            newJointPositions = app.myNiryoOne.model.ikcon(targetPose, currentJointPositions);

            app.myNiryoOne.model.animate(newJointPositions);
            [app.myNiryoOneLink1, app.myNiryoOneLink2, app.myNiryoOneLink3, app.myNiryoOneLink4, app.myNiryoOneLink5, app.myNiryoOneLink6] = deal(newJointPositions(1), newJointPositions(2), newJointPositions(3), newJointPositions(4), newJointPositions(5), newJointPositions(6));
        end

        % Button pushed function: MoveUR3Button
        function MoveUR3ButtonPushed(app, event)
            xPos = app.XPosEditField_2.Value;
            yPos = app.YPosEditField_2.Value;
            zPos = app.ZPosEditField_2.Value;
            targetPose = transl(xPos, yPos, zPos);

            currentJointPositions = [app.myUR3Link1, app.myUR3Link2, app.myUR3Link3, app.myUR3Link4, app.myUR3Link5, app.myUR3Link6];
            newJointPositions = app.myUR3.model.ikcon(targetPose, currentJointPositions);

            app.myUR3.model.animate(newJointPositions);
            [app.myUR3Link1, app.myUR3Link2, app.myUR3Link3, app.myUR3Link4, app.myUR3Link5, app.myUR3Link6] = deal(newJointPositions(1), newJointPositions(2), newJointPositions(3), newJointPositions(4), newJointPositions(5), newJointPositions(6));
        end

        % [Other callback functions remain the same, using app.myUR3, app.myNiryoOne, and app.piggy]

    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)
            % [Your existing UI component creation code]
            % ... (omitted for brevity)
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = testGui(myNiryoOneInstance, myUR3Instance, piggyInstance, envInstance)

            % Store the robot instances passed from the main file
            app.myNiryoOne = myNiryoOneInstance;
            app.myUR3 = myUR3Instance;
            app.piggy = piggyInstance;
            app.env = envInstance;

            % Initialize the joint variables to the current robot positions
            currentNiryoJoints = app.myNiryoOne.model.getpos();
            [app.myNiryoOneLink1, app.myNiryoOneLink2, app.myNiryoOneLink3, app.myNiryoOneLink4, app.myNiryoOneLink5, app.myNiryoOneLink6] = deal(currentNiryoJoints(1), currentNiryoJoints(2), currentNiryoJoints(3), currentNiryoJoints(4), currentNiryoJoints(5), currentNiryoJoints(6));

            currentUR3Joints = app.myUR3.model.getpos();
            [app.myUR3Link1, app.myUR3Link2, app.myUR3Link3, app.myUR3Link4, app.myUR3Link5, app.myUR3Link6] = deal(currentUR3Joints(1), currentUR3Joints(2), currentUR3Joints(3), currentUR3Joints(4), currentUR3Joints(5), currentUR3Joints(6));

            % Create UIFigure and components
            createComponents(app);

            % Register the app with App Designer
            registerApp(app, app.UIFigure);

            % Execute the startup function
            runStartupFcn(app, @startupFcn);

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)
            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end

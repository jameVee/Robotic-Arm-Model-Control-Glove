classdef tgrRobotEnvInteraction < handle
% Based on exampleHelperCoordinatorPickPlace
% Copyright 2020 The MathWorks, Inc.

    properties
        % robot properties
        Robot               % robot 
        RobotEndEffector    % gripper
        CurrentJointConfig  % joint configuration
        CurrentTaskConfig   % task configuration
        MotionModel         %
        NumJoints           % number of joints
        % environment properties
        Environment         % environment
        DetectedParts = {}; 
        % interaction properties
        Controller          % controller 
        % simulation properties
        Figure              % visualization figure
        % operation properties
        NextPart = 0;
        PartOnRobot = 0;
        HomeRobotTaskConfig         
    end
    
    methods
        function obj = tgrRobotEnvInteraction(robot, joint_config, end_effector, env)
            % robot properties
            obj.Robot = robot;            
            obj.CurrentJointConfig = joint_config;
            obj.RobotEndEffector = end_effector;
            obj.CurrentTaskConfig = getTransform(obj.Robot, obj.CurrentJointConfig, obj.RobotEndEffector);
            obj.MotionModel = jointSpaceMotionModel('RigidBodyTree', obj.Robot);
            obj.NumJoints = numel(obj.CurrentJointConfig);

            % environment properties
            obj.Environment = env;
                    
            % visualization properties
            obj.Figure = interactiveRigidBodyTree(obj.Robot,'ShowMarker',false, 'Frames', 'off'); 
            obj.Figure.Configuration = obj.CurrentJointConfig;
            obj.Figure.ShowMarker = false;
            hold on
            axis([-1 1 -1 1 -0.1 1.5]);
            view(58,25);            
        end
        
        % visualize workstation
        function visualizeStation(obj)
            try
                bench = obj.Environment.Station{1};
                beltL = obj.Environment.Station{2};
                beltR = obj.Environment.Station{3};
            
                [~, p1] = show(bench);
                [~, p2] = show(beltL);
                [~, p3] = show(beltR);

                p1.FaceColor = [0.5 0.5 0.5];
                p1.FaceAlpha = 1.0;
                p1.LineStyle = 'none';

                p2.FaceColor = [0.7,0.7,0];
                p2.FaceAlpha = 1.0;
                p2.LineStyle = 'none';

                p3.FaceColor = [0,0.7,0.7];
                p3.FaceAlpha = 1.0;  
                p3.LineStyle = 'none';          

                drawnow;
            catch
                disp('Error');
            end
        end

        % visualize parts
        function visualizeParts(obj)
            for i = 1:length(obj.Environment.Parts)
                tempPose = [0,0,0]; % to set transformation reference
                correctPose = obj.Environment.Parts{i}.mesh.Pose;
                obj.Environment.Parts{i}.mesh.Pose = trvec2tform(tempPose);
                [~, obj.Environment.Parts{i}.plot] = show(obj.Environment.Parts{i}.mesh);
                obj.Environment.Parts{i}.plot.LineStyle = 'none'; 
                obj.Environment.Parts{i}.plotHandle = hgtransform;
                obj.Environment.Parts{i}.plot.Parent = obj.Environment.Parts{i}.plotHandle;
                obj.Environment.Parts{i}.mesh.Pose = correctPose;
                obj.Environment.Parts{i}.plotHandle.Matrix = obj.Environment.Parts{i}.mesh.Pose;
                obj.Environment.Parts{i}.plot.FaceColor = obj.Environment.Parts{i}.color; 
            end
            drawnow;
        end

        % visualize robot
        function moveJoints(obj, joint_states)
            obj.Figure.Configuration = joint_states;
            obj.Figure.ShowMarker = false;
            % Update current robot configuration
            obj.CurrentJointConfig = joint_states;
            obj.CurrentTaskConfig = getTransform(obj.Robot, obj.CurrentJointConfig, obj.RobotEndEffector);
            % Visualize parts
            if obj.PartOnRobot ~= 0
                obj.Environment.Parts{obj.PartOnRobot}.mesh.Pose = obj.CurrentTaskConfig * trvec2tform([0 0 0.04]);
                obj.Environment.Parts{obj.PartOnRobot}.plotHandle.Matrix = obj.Environment.Parts{obj.PartOnRobot}.mesh.Pose;
            end
            drawnow;            
        end

        % Delete function
        function delete(obj)
            delete(obj.Controller)
        end
            
    end
end


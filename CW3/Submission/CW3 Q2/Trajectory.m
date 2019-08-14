function Trajectory(scene, clientID, vrep, handles, Youbot, setPoseArmPub, closeGripperPub, openGripperPub)

    PRINT_TRAJ = true;
    
    if PRINT_TRAJ
        % Set Arm to zero
        for  point = 1 : 20
                SendJointTargetPosition(setPoseArmPub, [0,0,0,0,0]);
                pause(0.05); 
        end
    end
    
    while true
        % get_scene_data
        tic
        [data] = get_scene_data(scene, clientID, vrep, 'getPosOr', handles);
        toc
        if data.obj{1}.position(1) ~= 0 && data.stack{1}.position(1) ~= 0
            fprintf('Failed to Get data from VREP\n');
            break
        end
    end
    
    stack = data.stack{1};
    stack.size = 0;
    
    stack.pos = stack.position' +  [-0.07;-0.07; data.stack{1}.bbox_min(3)];
    
    for obj_num = 3:-1:1 
       
        % Sometimes VREP doesn't return any data - here we test using one value 
        % randomly and keep trying until it does
        while true
            % get_scene_data
            tic
            [data] = get_scene_data(scene, clientID, vrep, 'getPosOr', handles);
            toc
            if data.obj{1}.position(1) ~= 0
                break
            end
        end

        data.obj = {data.obj{obj_num}};
        
        % =============================================
        % Move Next Object
        % =============================================
        
        object_volume = data.obj{1}.bbox_max(1) *  data.obj{1}.bbox_max(2)* data.obj{1}.bbox_max(3);
        
        if object_volume < 0.0001
            [joint_data, stack] = create_trajectory(data, Youbot, stack);
            modes = size(joint_data,2);
        
            if PRINT_TRAJ
                for mode = 1:modes
                    [seg_points, joints] = size(joint_data{mode});
                    % Send trajectory to VREP
                    for  point = 1 : seg_points
                        SendJointTargetPosition(setPoseArmPub, joint_data{mode}(point,:));
                        pause(0.05); 
                    end

                    % Choose gripper action according to the mode that has just
                    % been completed (staging, pick-up, put-down, zero)
                    if mode == 1
                        % Open gripper in preparation for grasping
                        GripperAction(openGripperPub);
                        pause(0.5);
                    elseif mode == 2
                        % Close gripper on object
                        GripperAction(closeGripperPub);
                        pause(0.5);
                    elseif mode == 4
                        % Close gripper on object
                        GripperAction(openGripperPub);
                        pause(0.5);
                    end
                end
            end
        end
    end
end


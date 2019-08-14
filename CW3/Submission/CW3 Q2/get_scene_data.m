%%
% @data:    1st row is object structs, second row is obstacle structs
function [data] = get_scene_data(query, clientID, vrep, mode, handles)

    fields = fieldnames(query);
    num_fields = size(fields,1);
    
    for field_num = 1:num_fields
        
        % Field name
        field_name = char(fields(field_num));
        
        % Check if any scene items need to be fetched for this field type
        if isempty(query.(field_name))
            continue
        end
        
        num_items = size(query.(field_name),2);
        data.(field_name) = cell(1, num_items);
        
        for item_num = 1:num_items
            
            item_name = char(query.(field_name)(item_num));
            
            % Get handles for objects in the scene
            if strcmp(mode, 'getHandles')
                
                [~, obj] = vrep.simxGetObjectHandle(clientID, item_name, vrep.simx_opmode_blocking);
                data.(field_name){item_num} = obj;
                
            % Get object position and orientation data
            else
                handle = handles.(field_name){item_num};
                data.(field_name){item_num} = GetObjectPosAndOrientation(clientID, vrep, item_name, handle);

                if strcmp(field_name, 'obj') || strcmp(field_name, 'obs') || strcmp(field_name, 'youbot')

                    % Append rotation matrix
                    data.(field_name){item_num}.R = rot_mat(data.(field_name){item_num}.orientation(1),...
                                            data.(field_name){item_num}.orientation(2), ...
                                            data.(field_name){item_num}.orientation(3));
                end
            end
        end
        
    end
    
    % ================================
    % Add arm origin field
    % ================================
    
    % Get handles for objects in the scene
    if strcmp(mode, 'getPosOr')
        data.('arm_origin') = (data.('youbot'){1}.position)';
        % Correct for the position of the youbot origin
        youbot_z = data.('youbot'){2}.position(3) - 0.147;
        data.('arm_origin')(3) = youbot_z;
    end
end


%%
% Subroutine to compute rotation from Euler angles
% Takes the roll pitch and yaw angles (a,b,g) for rotation about world
% coordinates
function R = rot_mat(a,b,g)
    R_x = rotx(a);
    R_y = roty(b);
    R_z = rotz(g);

    R = R_x * R_y * R_z;
    R = R;
end
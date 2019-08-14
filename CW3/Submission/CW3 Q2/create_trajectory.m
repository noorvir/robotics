%%
%
% yba refers to youbot-arm frame (base of joint 0)
function [joint_data, stack] = create_trajectory(data, robot, stack)

    % starting position
    joint_angles_zero = [0.0,0.0,0.0,0.0,0];
    % Number of samples per 3 seconds interval
    N = 150;
    timing.t0 =  0;
    timing.tf =  3;
    timing.t_step = timing.tf/N;
    p=1;

    % Distance of staging position from object
    STAGE_TOL = 0.06;
    % Distance to move end-effector forward by in addition to object position 
    % in order to grip object
    GRIP_TOL = 0.01;
    % Offset from Rectangle13 position to place stack of objects. (In
    % Youbot frame)
    X_P_OFFSET = -0.1;
    Y_P_OFFSET = -0.03;
    
    % ==========================================
    % Constraints
    % ==========================================
    % Velocity constraints
    cnst.vel(1) = 0;
    cnst.vel(2) = 0;
    % Acceleration constraints
    cnst.acc(1) = 0;
    cnst.acc(2) = 0;
    
    % ==========================================
    % Initialise variables
    % ==========================================
    joint_profile = [];%zeros(200,5);
    num_obj = numel(data.obj);
    num_obs = numel(data.obs);
    % Total data-points in entire trajectory
    total_points = N * (num_obj - 1) + 1;
    % Transformation matrix of each point in trajectory
    tHom = zeros(4,4,total_points);
    % Coefficients for all polynomial between points - 6 coefficients for
    % each dimension (x,y,z) for num_points-1 polynomials
    pw_pol_coefs = zeros(5,6,2*num_obj);
    % Objects and obstacle locations in world frame
    T_obj_w = zeros(4,4,num_obj);
    T_obs_w = zeros(4,4,num_obs);
    % Objects and obstacle locations in you-bot arm frame
    T_obj_yba = zeros(4,4,num_obj);
    T_obs_yba = zeros(4,4,num_obs);
    
    
    % ==========================================
    % Transformations
    % ==========================================
    % Youbot base position in world coordinate
    yb_pos_w = data.arm_origin;
    % Rotation of youbot arm frame in world frame (mainly yaw)
    yb_joint_rec_w = data.youbot{1}.R;
    R_yba_w = rotz(pi/2) * yb_joint_rec_w;
    % Youbot-arm frame in world coordinates
    T_yba_w = [R_yba_w, yb_pos_w; 0 0 0 1];
    % World coordinates in youbot-arm frame 
    T_w_yba = inv(T_yba_w);
    
    % ==========================================
    % Scene Description
    % ==========================================
    % Get postion and orientation of objects in world co-ordinates as well 
    % as Youbot arm base coordinates
    for obj = 1:num_obj
        obj_pos_w = data.obj{obj}.position';
        T_obj_w(:,:,obj) = [data.obj{obj}.R, obj_pos_w; 0 0 0 1];
        % Pose in yba frame
        T_obj_yba(:,:,obj) = T_w_yba * T_obj_w(:,:,obj);
    end
    
    % Get postion and orientation of obstacles in world co-ordinates as well 
    % as Youbot arm base coordinates
    for obs = 1:num_obs
        obs_pos_w = data.obs{obs}.position';
        T_obs_w(:,:,obs) = [data.obs{obs}.R, obs_pos_w; 0 0 0 1];
        % Pose in yba frame
        T_obs_yba(:,:,obs) = T_w_yba * T_obs_w(:,:,obs);
    end
            
    % =============================================
    % Platform Positional Properties
    % =============================================
    % x and y coordinates of the center of the top most block on the stack
    s_pos_xy_w = stack.pos(1:2);
    % Tranformation matrix for stack loaction in youbot coordinates
    s_pos_yba = T_w_yba * [s_pos_xy_w; stack.pos(3);1];
    s_pos_yba = s_pos_yba(1:3);
%     s_pos_yba = s_pos_yba + [X_P_OFFSET; Y_P_OFFSET;0;0]; 
    % Angle in the xy-plane between the direction of the object and the
    % x-axis of the youbot arm
    angle_to_platform = acos(dot([1;0],s_pos_yba(1:2))/(norm(s_pos_yba(1:2))*norm([1,0])));
    % Resolve acos direction 
    direction_a_p = sign(sum(cross([1;0;0],[s_pos_yba(1:2);0])));
    angle_to_platform = direction_a_p *  angle_to_platform;
    angle_downward = deg2rad(150);
    stack.tHom =  [[rotz(angle_to_platform) * roty(angle_downward); [0 0 0]], [s_pos_yba;1]];

    % =============================================
    % Object Positional Properties
    % =============================================

    des_trans = T_obj_yba(1:3,4,p);
    % Position vector (xy-plane) of object in youbot frame
    ob_xy_vec = des_trans(1:2);
    % Angle in the xy-plane between the direction of the object and the
    % x-axis of the youbot arm
    angle_to_object = acos(dot([1;0],ob_xy_vec)/(norm(ob_xy_vec)*norm([1,0])));
    % Resolve acos direction 
    direction_a_o = sign(sum(cross([1;0;0],[ob_xy_vec;0])));
    angle_to_object = direction_a_o *  angle_to_object;


    if angle_to_object > 0
        angle_to_object  = angle_to_object + deg2rad(4);
    else
        angle_to_object  = angle_to_object - deg2rad(4);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%STAGE%%%%%%%%%%%%%%%%%%

    [t1, REV] = stage_angle(angle_to_object);
    stage_theta = [t1, 0,0,0,0];

    % Postion constraints
    points(:,1) = joint_angles_zero;
    points(:,2) = stage_theta;

    pw_pol_coefs(:,:,1) = fit_polynomial('quintic', points, cnst, timing.t0, timing.tf);

    point = 1;
    % Sample the quintic curve every 3/N time steps to get end-effector
    % tip translations
    [joint_profile, ~] = sample_polynomial(joint_profile, ...
                                                pw_pol_coefs, 1, timing.t0,...
                                                timing.t_step, timing.tf, point);
    joint_data{1} = joint_profile;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%PICK-UP%%%%%%%%%%%%%%%%%%

    object_volume = data.obj{p}.bbox_max(1) * data.obj{p}.bbox_max(2) * data.obj{p}.bbox_max(3);
    % If it is the big object, grab it by it top corner
    if object_volume < 0.0001
        des_trans = T_obj_yba(1:3,4,p);
        % Calculate staging xy-position
        del_x = sign(des_trans(1)) * 0.005 * abs(cos(angle_to_object));
        del_y = sign(des_trans(2)) * 0.005 * abs(sin(angle_to_object));
        des_trans = T_obj_yba(1:3,4,p);
        des_trans = des_trans + [del_x; del_y; 0.004];
    else
        des_trans = T_obj_yba(1:3,4,p);
        % Calculate staging xy-position
        del_x = -sign(des_trans(1)) * 0.045 * abs(cos(angle_to_object));
        del_y = -sign(des_trans(2)) * 0.045 * abs(sin(angle_to_object));
        des_trans = des_trans + [del_x; del_y;0.025];
    end

    if REV
        angle_pickup = deg2rad(120);
        pickup_tHom = [rotz(angle_to_object)*roty(angle_pickup)*rotz(pi), des_trans;[0 0 0 1]];
    else
        angle_pickup = deg2rad(90);
        pickup_tHom = [rotz(angle_to_object)*roty(angle_pickup), des_trans;[0 0 0 1]];
    end

    theta_inc = deg2rad(5);
    break_out = false;
    while true
        pickup_t = inverse_kinematics(pickup_tHom, robot);
        for i =4:-1:1
            if ~any(isnan(pickup_t(i,:))) && any(pickup_t(i,:))
                pickup_theta = pickup_t(i,:);
                break_out = true;
                break
            end
        end
        if break_out
            break
        end

        angle_pickup = angle_pickup + sign(t1)*theta_inc;

        if REV
            pickup_tHom = [rotz(angle_to_object)*roty(angle_pickup)*rotz(pi), des_trans;[0 0 0 1]];
        else
            pickup_tHom = [rotz(angle_to_object)*roty(angle_pickup), des_trans;[0 0 0 1]];
        end
    end

    % Postion constraints
    points(:,1) = stage_theta;
    points(:,2) = pickup_theta;

    pw_pol_coefs(:,:,1) = fit_polynomial('quintic', points, cnst, timing.t0, timing.tf);

    point = 1;
    % Sample the quintic curve every 3/N time steps to get end-effector
    % tip translations
    [joint_profile, ~] = sample_polynomial(joint_profile, ...
                                                pw_pol_coefs, 1, timing.t0,...
                                                timing.t_step, timing.tf, point);
    joint_data{2} = joint_profile;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% RETURN TO ZERO %%%%%%%%%%%%%%%%%%
    % Postion constraints
    points(:,1) = pickup_theta;
    points(:,2) = joint_angles_zero;

    pw_pol_coefs(:,:,1) = fit_polynomial('quintic', points, cnst, timing.t0, timing.tf);

    point = 1;
    % Sample the quintic curve every 3/N time steps to get end-effector
    % tip translations
    [joint_profile, ~] = sample_polynomial(joint_profile, ...
                                                pw_pol_coefs, 1, timing.t0,...
                                                timing.t_step, timing.tf, point);
    joint_data{3} = joint_profile;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%DROP-CUBE%%%%%%%%%%%%%%

    stack_t = inverse_kinematics(stack.tHom, robot);

    for i =4:-1:1
        if ~any(isnan(stack_t(i,:))) && any(stack_t(i,:))
            stack_theta = stack_t(i,:);
            break
        end
    end
    
    theta_inc = deg2rad(5);
    break_out = false;
    angle_pickup = deg2rad(120);
    
    while true
        stack_t = inverse_kinematics(stack.tHom, robot);
        for i =4:-1:1
            if ~any(isnan(stack_t(i,:))) && any(stack_t(i,:))
                stack_theta = stack_t(i,:);
                break_out = true;
                break
            end
        end
        if break_out
            break
        end

        angle_pickup = angle_pickup + sign(t1)*theta_inc;

        if REV
            stack.tHom = [rotz(angle_to_object)*roty(angle_pickup)*rotz(pi), stack.pos;[0 0 0 1]];
        else
            stack.tHom = [rotz(angle_to_object)*roty(angle_pickup), stack.pos;[0 0 0 1]];
        end
    end
    
     % Postion constraints
    points(:,1) = joint_angles_zero;
    points(:,2) = stack_theta;

    pw_pol_coefs(:,:,1) = fit_polynomial('quintic', points, cnst, timing.t0, timing.tf);

    point = 1;
    % Sample the quintic curve every 3/N time steps to get end-effector
    % tip translations
    [joint_profile, ~] = sample_polynomial(joint_profile, ...
                                                pw_pol_coefs, 1, timing.t0,...
                                                timing.t_step, timing.tf, point);
    joint_data{4} = joint_profile;


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%% RETURN TO ZERO %%%%%%%%%%%%%%%%%%
    % Postion constraints
    points(:,1) = stack_theta;
    points(:,2) = joint_angles_zero;

    pw_pol_coefs(:,:,1) = fit_polynomial('quintic', points, cnst, timing.t0, timing.tf);

    point = 1;
    % Sample the quintic curve every 3/N time steps to get end-effector
    % tip translations
    [joint_profile, ~] = sample_polynomial(joint_profile, ...
                                                pw_pol_coefs, 1, timing.t0,...
                                                timing.t_step, timing.tf, point);
    joint_data{5} = joint_profile;
    
    
    s_height = stack.tHom(3,4) + 2 * data.obj{p}.bbox_max(3) + 0.001;
    stack.pos = stack.pos + [0;0;s_height];
end
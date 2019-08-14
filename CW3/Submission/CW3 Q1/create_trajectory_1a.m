function [joint_profile, velocity_profile] = create_trajectory_1a(data)
    [num_points, num_joints] = size(data);
    % Number of samples per 3 seconds interval
    N = 20;
    num_output = N * (num_points-1) + 1;
    % Output matrices
    joint_profile = zeros(num_output, 5);
    velocity_profile = zeros(num_output, 5);
    
    %%%%% Initialise constraints %%%%%
    
    % Velocity constraints
    v_i = zeros(num_joints, 1);
    v_f = zeros(num_joints, 1);

    % Acceleration constraints
    a_i = zeros(num_joints, 1);
    a_f = zeros(num_joints, 1);
    
    % Point counter for array of intermediate poses
    point = 1;
    
    for traj_point = 1:num_points-1
        
        t0 =  (traj_point - 1) * 3;
        t_step = 3/N;
        tf =  traj_point * 3;
        
        %%%%% Initialise position constraints %%%%%

        % Postion constraints
        q_i = data(traj_point,:)';
        q_f = data(traj_point+1,:)';

        % Polynomial coefficients for each joint
        q_coeff = zeros(num_joints, 6);
        
        
        %%%%% Initialise quintic polynomial %%%%%

        % Coefficient matrix
        A = [1, t0, t0^2, t0^3, t0^4, t0^5;...
             0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;...
             0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;...
             1, tf, tf^2, tf^3, tf^4, tf^5;...
             0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;...
             0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

        for j = 1:num_joints
            % Constraints, b
            b = [q_i(j);v_i(j);a_i(j);q_f(j);v_f(j);a_f(j)];

            % Variables to solve for, x
            x = linsolve(A,b);
            
            % Store polynomial coefficients for each joint in the column of
            % the matrix
            q_coeff(j,:) = x';
        end
        
        % Prevent sampling the same time twice
        if traj_point > 1
            t0 = t0 + t_step;
        end
        
        % sample the quintic curve every 3/N time steps.
        for t = t0 : t_step : tf
            % Evalute joint profile from polynomial at current time step
            joint_profile(point, :) = [q_coeff(:,1) + q_coeff(:,2)*t + q_coeff(:,3)*t^2 + ...
                            q_coeff(:,4)*t^3 + q_coeff(:,5)*t^4 + q_coeff(:,6)*t^5]';
            
            % Evalute velocity profile from polynomial at current time step        
            velocity_profile(point,:) = [q_coeff(:,2) + 2*q_coeff(:,3)*t + 3*q_coeff(:,4)*t^2 + ...
                            4*q_coeff(:,5)*t^3 + 5*q_coeff(:,6)*t^4]'; 
            
            % Evalute velocity profile from polynomial at current time step        
            acceleration_profile(point,:) = [2*q_coeff(:,3) + 6*q_coeff(:,4)*t + ...
                            12*q_coeff(:,5)*t^2 + 20*q_coeff(:,6)*t^3]';
                        
            point = point + 1;
        end 
    end
    
    % Plot dynamic profiles
    t= 0:3/20:12;
    figure;
    plot(t,joint_profile);
    title('QUESTION 1a: Joint Angles');
    legend('J1','J2','J3','J4','J5')
    xlabel('Time')
    ylabel('Joint Angles')
    figure;
    plot(t,velocity_profile);
    title('QUESTION 1a: Joint Velocities');
    xlabel('Time')
    ylabel('Joint Velocities')
    legend('J1','J2','J3','J4','J5')
    
    figure;
    plot(t,acceleration_profile);
    title('QUESTION 1a: Joint Accelerations');
    xlabel('Time')
    ylabel('Joint Accelerations')
    legend('J1','J2','J3','J4','J5')
end
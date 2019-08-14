function [joint_profile, velocity_profile] = create_trajectory_1b(data)
    
    %Specify DH parameters each link of 4R-planar manipulator
    L1 = Link('d', 0.147, 'a', 0.033, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
    L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
    L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
    L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
    L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);

    %Construct the robot
    robot = SerialLink([L1, L2, L3, L4, L5]);
    
    % Number of points
    num_points = size(data,3);

    % Number of samples per 3 seconds interval
    N = 20;

    total_points = N * (num_points - 1) + 1;
    
    tHom = zeros(4,4,total_points);
    translation_profile = zeros(100,3);
    rotation_profile = double.empty(3,3,0);
    slerp_t = zeros(N+1,1);
    
    % Store all possible solutions for a particular configuration
    solutions = zeros(4,5,total_points);
    
    % Rotation Components
    R(:,:,1:5) = data(1:3,1:3,:);
    % Translation Components 
    tr(:,:,1:5) = data(1:3,4,:);
    tr = squeeze(tr);
    
    % Velocity constraints
    cnst.vel(1) = 0;
    cnst.vel(2) = 0;
    % Acceleration constraints
    cnst.acc(1) = 0;
    cnst.acc(2) = 0;
   
    % Coefficients for all polynomial between points - 6 coefficients for
    % each dimension (x,y,z) for num_points-1 polynomials
    pw_pol_coefs = zeros(3,6,num_points-1);

    % Point counter for array of intermediate poses
    point_t = 1;
    point_r = 1;
    
    for p = 1:num_points-1
        
        % Timing 
        t0 =  (p - 1) * 3;
        t_step = 3/N;
        tf =  p * 3;
        
        %===========================================
        % Translations
        %===========================================
        
        % Postion constraints
        points(:,1) = tr(:,p);
        points(:,2) = tr(:,p+1);

        % Fit polynomial and store the piece-wise polynomial coefficients
        % in a matrix
        pw_pol_coefs(:,:,p) = fit_polynomial('quintic', points, cnst, t0, tf);
        
        
        % Sample the quintic curve every 3/N time steps to get end-effector
        % tip translations
        [translation_profile, point_t] = sample_polynomial(translation_profile, ...
                                                        pw_pol_coefs, p, t0,...
                                                        t_step, tf, point_t);
        
        %===========================================
        % Orientations
        %===========================================
        
        t_step_slerp = 1/N;

        % Only generate/sample slerp parameter once
        if point_r == 1
            
            t0_slerp = 0;

            % Fit a quintic polynomial to the SLERP time parameter t
            slerp_t_coefs = fit_polynomial('quintic', [0,1], cnst, 0, 1);

            % Calculate slerp parameter (t) values
            [slerp_t, point_r] = sample_polynomial(slerp_t, slerp_t_coefs, 1, t0_slerp,...
                                                 t_step_slerp, 1, point_r);
        end
        
        % Prevent sampling the same time twice
        if p == 2
            slerp_t = slerp_t(2:end);
        end

        % Convert rotation matrices to quaternions
        q1 = Quaternion(R(:,:,p));
        q2 = Quaternion(R(:,:,p+1));
        
        % Sompute the angle between the two vectors in quaternion space
        theta = acos(abs(q1.s * q2.s + dot(q1.v,q2.v)));
        
        % Cover corner case to avoid taking longer path when q1.q2 < 0.
        % This also prouces errors in inverse kinematics if not handled
        if (q1.s * q2.s + dot(q1.v,q2.v)) < 0
            q2 = -1 * q2;
        end
        
        % Calculate interpolated rotation points
        rotation_profile = slerp(rotation_profile,slerp_t, theta, q1, q2);   
    end
    
    % Arrange rotations and translations as transformation matrices
    for i = 1:total_points
        
        tHom(:,:,i) = [rotation_profile(:,:,i), translation_profile(i,:)'; 0,0,0,1];
        solutions(:,:,i) = inverse_kinematics(tHom(:,:,i), robot);
        joint_profile(i,:) = solutions(4,:,i);
        
        if i > 1
            velocity_profile(i,:) = joint_profile(i,:) - joint_profile(i-1,:);
            acceleration_profile(i,:) = velocity_profile(i,:) - velocity_profile(i-1,:);
        else
            velocity_profile(i,:) = joint_profile(i,:) - joint_profile(i,:);
            acceleration_profile(i,:) = velocity_profile(i,:) -  velocity_profile(i,:);
        end
    end
    
    
    t= 0:3/20:12;
    
    figure;
    plot(t,joint_profile(:,:))
    xlabel('Time')
    ylabel('Joint Angles')
    title('QUESTION 1b: Joint Angles');
    legend('J1','J2','J3','J4','J5')
    
    figure;
    plot(t,velocity_profile);
    title('QUESTION 1b: Joint Velocities');
    xlabel('Time')
    ylabel('Joint Velocities')
    legend('J1','J2','J3','J4','J5')
    
    figure;
    plot(t,acceleration_profile);
    title('QUESTION 1b: Joint Accelerations');
    xlabel('Time')
    ylabel('Joint Accelerations')
    legend('J1','J2','J3','J4','J5')
    
    
end
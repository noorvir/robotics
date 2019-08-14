function [joint_profile, velocity_profile] = create_trajectory_1c(data)
    
    %Specify DH parameters each link of 4R-planar manipulator
    L1 = Link('d', 0.147, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
    L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
    L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
    L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
    L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);

    %Construct the robot
    robot = SerialLink([L1, L2, L3, L4, L5]);

    [M, N] = size(data);
    
    N = 20;
    num_points = size(data,3);
    
    theta = zeros(num_points, 5);
    
    solutions = zeros(4,5,num_points);
    
    
    for p = 1:num_points
        
        solutions(:,:,p) = inverse_kinematics(data(:,:,p), robot);
        
        if p <= 3
            theta(p,:) = solutions(4,:,p);
        else
            theta(p,:) = solutions(1,:,p);
        end
        
    end
    
    % Point in joint_profile
    point = 1;
    
    % Create joint profile to interpolate between all target points
    for p = 1:num_points-1
        
        % Start and stop times for current point
        t0 = 0;
        t_step = 3/N;
        tf = 3;
        
        % Constant velocity start-time
        t_s = t0 + 1;
        % constant velocity end-time
        t_e = t0 + 2;
         
       % ====================================== 
       % Fit polynomial
       % ======================================
       
       % Velocity constraint
       V = (theta(p+1,:) - theta(p,:))/(tf - t_s);
       
       % Intercept for constant velocity phase (linear positional change)
       c = 0.5 * (theta(p,:) + theta(p+1,:) - V*tf);
       
       % Acceleration Polynomial
       a0 = theta(p,:)';
       a1 = zeros(5,1);
       a2 = V'/(2*t_s);
       
       % Decceleration Polynomial
       a0_d = theta(p+1,:)';
       
       % Solve for coefficients
       A = [2*tf, 1;...
           (tf-t0)^2, (tf-t0)];
       b = [zeros(1,5);-theta(p+1,:) + V];
       x = linsolve(A,b);
       
       a1_d = x(1,:)';
       a2_d = x(2,:)';
       
       % Store polynomial coefficients for each joint in the column of
       % the matrix - acceleration phase and deceleration phase
       q_coeff_a = [a2,a1,a0];
%        q_coeff_d = [a2_d,a1_d,a0_d];

       al = V/t_s;
    
       % sample both the constant/non-constant velocity polynomials every 
       % 3/N time steps.
       for t = 0:t_step:3
            
            % Acceleration step
            if t <= t_s
               joint_profile(point,:) = (q_coeff_a(:,1) * t^2 + ...
                                        q_coeff_a(:,2)*t + q_coeff_a(:,3))';
               
            % Constant velocity step   
            elseif t_s < t && t <= t_e
            
                joint_profile(point,:) = (V*t + c)';
                
            % Decceleration step
            else
                  joint_profile(point,:) = theta(p+1,:) - al*(0.5*t^2 + 0.5*tf^2 ...
                                                             - tf*t);
            end
            
            if point > 1
                velocity_profile(point,:) = joint_profile(point,:) - joint_profile(point-1,:);
            else
                velocity_profile(point,:) = joint_profile(point,:) - joint_profile(point,:);
            end
            point = point + 1;
        end 
    end
    
    
    t= 0:12/84:12;
    
    figure;
    plot(t(1:84),joint_profile(:,:))
    title('QUESTION 1c: Joint Angles');
    xlabel('Time')
    ylabel('Joint Angles')
    title('QUESTION 1c: Joint Angles');
    legend('J1','J2','J3','J4','J5')
    
    figure;
    plot(t(1:84),velocity_profile);
    title('QUESTION 1c: Joint Velocities');
    xlabel('Time')
    ylabel('Joint Velocities')
    legend('J1','J2','J3','J4','J5')
end
%%
% This function computes the series of joint angles requrired to trace a
% trajectory
% desired_traj  -   Array of pose vectors defined by the Rodrigues
%                   convention
% robot         -   object of SerialLink type describing the physical 
%                   configuration of the manipulator
% option        -   The question number to execute
function theta = inverse_kine_jacobian(desired_traj, robot, option)
    [M, N] = size(desired_traj);
    
    % Tolerance for break condition of solver in meters
    tol = 0.01;
    % Initial joint angle values
    theta = zeros(M,robot.n);
    thetaInt = zeros(1,robot.n);    
        
    % Loop through all trajectory points
    for tjPoint = 1: M

        % Question 1
        if (strcmp(option, 'q1'))
            
            % Mask defining Cartesian DoF of end-effector
            poseMask = [1 1 1 0 0 0];

            % End-effector position as a homogenous matrix
            ee_x = desired_traj(tjPoint, 1);
            ee_y = desired_traj(tjPoint, 2);
            eeCoords = transl(ee_x, ee_y, 0);

        % Question 5
        elseif (strcmp(option, 'q4'))
            
            % Mask defining Cartesian DoF of end-effector
            poseMask = [1 1 1 1 1 0];

            pose = desired_traj(tjPoint,:);
            % Convert pose into homogenous transformation matrix
            rot = rodrigues(pose(4:6));
            eeCoords = [rot, pose(1:3)'; [0 0 0 1]];
        
        % Question 6
        elseif (strcmp(option, 'qextra'))
            theta = zeros(M, 7);
        end

        % Compute inverse transformation
        theta(tjPoint, :) = cInvKinePseudoJacobian(robot,...
                                                    eeCoords,...
                                                    thetaInt,...
                                                    tol,...
                                                    poseMask);
        thetaInt = theta(tjPoint,:);
    end
        
end






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
    
    % Question 1
    if (strcmp(option, 'q1'))
        
        % Mask defining Cartesian DoF of end-effector
        poseMask = [1 1 1 0 0 0];
        
        % Loop through all trajectory points
        for tjPoint = 1: M
            % End-effector position as a homogenous matrix
            ee_x = desired_traj(tjPoint, 1);
            ee_y = desired_traj(tjPoint, 2);
            eeCoords = transl(ee_x, ee_y, 0);
            % Compute inverse transformation 
            theta(tjPoint, :) = cInvKinePseudoJacobian(robot,...
                                                        eeCoords,...
                                                        thetaInt,...
                                                        tol,...
                                                        poseMask);
            thetaInt = theta(tjPoint,:);
        end
        
    % Question 5
    elseif (strcmp(option, 'q4'))
        
        % Loop through all trajectory points
        for tjPoint = 1: M
            
            pose = desired_traj(tjPoint,:);
            % Convert pose into homogenous transformation matrix
            rot = rodrigues(pose(4:6));
            eeCoords = [rot, pose(1:3)'; [0 0 0 1]];
            
            % Mask defining Cartesian DoF of end-effector
            poseMask = [1 1 1 1 1 0];
            
            % Compute inverse transformation 
            theta(tjPoint, :) = cInvKinePseudoJacobianQ4(robot,...
                                                        eeCoords,...
                                                        thetaInt,...
                                                        tol,...
                                                        poseMask);
            thetaInt = theta(tjPoint,:);
        
        end
    % Question 6
    elseif (strcmp(option, 'qextra'))
        theta = zeros(M, 7);
    end
end






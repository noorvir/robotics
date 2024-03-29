%%
% This function computes the series of joint angles requrired to trace a
% trajectory
% desired_traj  -   Array of pose vectors defined by the Rodrigues
%                   convention
% robot         -   object of SerialLink type describing the physical 
%                   configuration of the manipulator
% use_CF_q4     -   option to use analytical solution derived in q4.b. Set
%                   this to false for q1
% option        -   The question number to execute
function theta = inverse_kine_jacobian(desired_traj, robot, use_CF_q4, option)
    
    fprintf('Calculating inverse kinematic soltion: Jacobian Method... \n');
    
    limit_angles = true;
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
            
            % Joint limits array
            limits =    [deg2rad(-169) deg2rad(169);
                          deg2rad(-65) deg2rad(90);
                          deg2rad(-151) deg2rad(146);
                          deg2rad(-102.5) deg2rad(102.5);
                          deg2rad(-167.5) deg2rad(167.5);];
        
        % Question 6
        elseif (strcmp(option, 'qextra'))
            % Mask defining Cartesian DoF of end-effector
            poseMask = [1 1 1 1 1 1];
            
            pose = desired_traj(tjPoint,:);
            % Convert pose into homogenous transformation matrix
            rot = rodrigues(pose(4:6));
            eeCoords = [rot, pose(1:3)'; [0 0 0 1]];
            
            % Joint limits array
            limits =    [deg2rad(-169) deg2rad(169);
                          deg2rad(-65) deg2rad(90);
                          deg2rad(-151) deg2rad(146);
                          deg2rad(-102.5) deg2rad(102.5);
                          deg2rad(-167.5) deg2rad(167.5);];
            
        end

        % Compute inverse transformation
        theta(tjPoint, :) = cInvKinePseudoJacobian(robot,...
                                                    eeCoords,...
                                                    thetaInt,...
                                                    tol,...
                                                    poseMask,...
                                                    use_CF_q4);
        
        % If limit_angles flag is set, maximum joint angle value will be
        % limited to manipulator range
        if limit_angles
            
            thetaOld = theta(tjPoint,:);
            
            if (strcmp(option, 'qextra') || strcmp(option, 'q4'))                                     
                for jt = 1:robot.n
                    [theta(tjPoint,jt),thetaF] =  checkJointLimits(theta(tjPoint,jt), jt, limits);
                    
                    % Display warning if joint angle had to be limited
                    if thetaOld(jt) ~= theta(tjPoint,jt)
                        fprintf('WARNING: Joint angle limit exceeded for joint %d . Angle forcefully limited to %3.3f . \n', jt, theta(tjPoint,jt));
                    end
                end 
            end
        end
        thetaInt = theta(tjPoint,:);
    end
    
        
end


% t5 = atan2(r11* sin(t1) - r21*cos(t1), r12*sin(t1) - r22*cos(t1))



%%
% This function computes a series of joint angles requrired to trace a
% trajectory
% desired_traj  -
% robot         -
% option        -
function theta = inverse_kine_jacobian(desired_traj, robot, option)
    [M, N] = size(desired_traj);
    
    if (strcmp(option, 'q1'))
        % Tolerance for break condition of solver in meters
        tol = 0.01;
        % Initial joint angle values
        theta = zeros(M,4);
        thetaInt = zeros(1,4);
        % Mask defining Cartesian DoF of end-effector
        poseMask = [1 1 1 0 0 0];
        
        
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
%             robot.plot(theta);
        end
        
    elseif (strcmp(option, 'q4'))
        theta = zeros(M, 5);
    elseif (strcmp(option, 'qextra'))
        theta = zeros(M, 7);
    end
end


%%
% This function calculates the inverse-kinematic solution for a given
% end-effector position using the Pseudo inverse of the Jacobian Matrix
% Arguments:
% robot         -   robot object of SerialLink type describing the
%                   manipulator
% eeCords       -   desired end-effector coordinates
% theta         -   initial values for joint angles
% tol           -   tolerance value to stop at
% poseMask      -   6x1 vector defining the cartesian DoF of the end-effector
function theta = cInvKinePseudoJacobian(robot, eeCoords, theta, tol, poseMask)
    
    % Translation component for the first frame
    Otransl0 = zeros(3, 1);
    % Axis of rotation for the first frame
    z0 = [0 0 1];
    % Jacobian Matrix
    jMat = zeros(6, robot.n);
    % Desired rotation and translation components of the ee position
    [dRotEE, dTranslEE] = tr2rt(eeCoords);
    % 4x1 Pose vectors - we ignore the rotations about x and y-axes 
    pose = zeros(6,1);
    dPose = zeros(6,1);
    dPose(1:3) = dTranslEE;
    dPose(4:6) = tr2rpy(dRotEE);
    % Only keep the Cartesian pose variables specified in poseMask
    dPose = dPose(poseMask == 1);
    % Workspace radius
    maxExtensionLen = norm(subIndex(robot.fkine(zeros(1, robot.n)),1:3,4));
    % Closest the ee can get to the desired position given the current
    % workspace
    eeBestApproachError = norm(dTranslEE) - norm(maxExtensionLen);
    
    while true
        % Current rotation and translation components of the ee position
        [cRotEE,OtranslEE] = tr2rt(robot.fkine(theta));
        % Current ee pose
        pose(1:3) = OtranslEE;
        pose(4:6) = tr2rpy(cRotEE);

        for j = 1: robot.n
           
           % Use special case for the first joint/frame. This might change
           % based on the robot geometry. For example when the first joint
           % is not at the base
           if (j == 1)
               % Update the linear velocity components
               jMat(1:3,j) = cross(z0, OtranslEE -  Otransl0);
               % Update angular velocity component
               jMat(4:6,j) = z0;
           else
               % Axis of rotation j-1
               z = subIndex(robot.A(1:j-1, theta),1:3,robot.n-1);
               % Translation component of frame from origin
               Otransl = subIndex(robot.A(1:j-1, theta), 1:3, 4);
               % Update the linear velocity components
               jMat(1:3,j) = cross(z, OtranslEE -  Otransl);
               % Update angular velocity component
               jMat(4:6,j) = z;
           end
        end
       
        % Only keep the dimensions we care about
        pose = pose(poseMask == 1);
        jMat = jMat(poseMask == 1, :);

        % Compute inverse-kinematics for target ee - position
        theta = theta + (pinv(jMat) * (dPose - pose))';

        cTranslEE = subIndex(robot.fkine(theta), 1:3, 4);
        eePosOffset = norm(cTranslEE - dTranslEE); 
       
        % Break condition
        if eePosOffset < tol
            % If desired ee position lies within robot workspace
            break;
        elseif eeBestApproachError > 0 && ...
                    (eePosOffset < eeBestApproachError + tol)
            % If desired ee position lies outside robot workspace and the
            % current position of end effector is the best the solver can
            % do (given the tolerance)
            break;
        end
       
    end
end


























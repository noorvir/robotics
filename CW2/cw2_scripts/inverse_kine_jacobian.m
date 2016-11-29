% function theta = inverse_kine_jacobian(desired_traj, robot, option)
%     [M, N] = size(desired_traj);
%     
%     if (strcmp(option, 'q1'))
%         theta = zeros(M, 4);
%         
%         
%         for tjPoint = 1: M
%             % -for every joint configuration, extract translation part of
%             % homogenous transformation for each joint
%             eeCords = [desired_traj(tjPoint,1), ...
%                        desired_traj(tjPoint,2), ...
%                        desired_traj(tjPoint,3)];
%             
%            calcJacobianMat(eeCoords, robot);
%             
%             eePosGlobD = transl(desired_traj(tjPoint,1), desired_traj(tjPoint,2), 0);
%             theta(tjPoint,:) = cInvKineCCD(eePosGlob, eePosGlobD, robot, tol);
%             eePosGlob = robot.fkine(theta(tjPoint,:));
%         end
%         
%         
%         % -find 0T4 for the end-effector using fikine
%         % -get all three joint positions 0T1 0T2 0T3 using robot.A
%         % -find the linear velocity part of the Jacobian using cross product
%         %  of the of axis of rottion of the previous joint and (O_n -
%         %  O_(i-1))
%         %  -get the angular velocity part of the Jacobian as the axis of rotation (in the planar case) 
%     elseif (strcmp(option, 'q4'))
%         theta = zeros(M, 5);
%     elseif (strcmp(option, 'qextra'))
%         theta = zeros(M, 7);
%     end
% end

% Vector of joint angles
theta = zeros(1, 4);
    
%Specify the length
L = 1;

%Specify DH parameters each link of 4R-planar manipulator
L1 = Link([0, 0, L, 0, 0], 'standard');
L2 = Link([0, 0, L, 0, 0], 'standard');
L3 = Link([0, 0, L, 0, 0], 'standard');
L4 = Link([0, 0, L, 0, 0], 'standard');

%Construct the robot
planar4Rrobot = SerialLink([L1, L2, L3, L4]);
eeCoords = transl(2,3,0);

jMat = cInvKinePseudoJacobian(planar4Rrobot,eeCoords, theta, 0.1)


%%
% This function calculates the inverse-kinematic solution for a given
% end-effector position using the Pseudo inverse of the Jacobian Matrix
% Arguments:
% robot         -   robot object of SerialLink type describing the
%                   manipulator
% eeCords       -   desired end-effector coordinates
% theta         -   initial values for joint angles
% to            -   tolerance value to at
function jMat = cInvKinePseudoJacobian(robot, eeCoords, theta, tol)
    
    % Translation component for the first frame
    Otransl0 = zeros(3, 1);
    % Axis of rotation for the first frame
    z0 = [0 0 1];
    % Jacobian Matrix
    jMat = zeros(6, robot.n);
    % Desired rotation and translation components of the ee position
    [dRotEE, dTranslEE] = tr2rt(eeCoords);
    % 4x1 Pose vectors - we ignore the rotations about x and y-axes 
    pose = zeros(4,1);
    dPose = zeros(4,1);
    dPose(1:3) = dTranslEE;
    dPose(4:6) = tr2rpy(dRotEE); 
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
               Otransl = subIndex(robot.A(1:j-1, theta), 1:3, robot.n);
               % Update the linear velocity components
               jMat(1:3,j) = cross(z, OtranslEE -  Otransl);
               % Update angular velocity component
               jMat(4:6,j) = z;
           end
        end
        
       % Compute inverse-kinematics for target ee - position
       theta = theta + (pinv(jMat) * (dPose - pose))';
       
       cTranslEE = subIndex(robot.fkine(theta), 1:3, 4);
       eePosOffset = norm(cTranslEE - dTranslEE); 
       
        robot.plot(theta);
        
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


























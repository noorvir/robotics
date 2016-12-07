
% %Specify DH parameters each link of 4R-planar manipulator
% L1 = Link('d', 0.147, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
% L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
% L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
% L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
% L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
% %Construct the robot
% Youbot = SerialLink([L1, L2, L3, L4, L5]);
% 
% 
% pose = [-0.0912500521904440,0.0174675124718185,0.437377970852528,-0.551410851752511,-0.0198774281345458,2.64347535753598];
% 
% rot = rodrigues(pose(4:6));
% eeCoords = [rot, pose(1:3)'; [0 0 0 1]];
% theta = zeros(1,5); 
% 
% plot3(pose(1),pose(2),pose(3), 'b*')
% % plot3( -0.127772293777214,0.0244587710530655,0.442124905491900, 'b*')
% xlabel('x')
% ylabel('y')
% grid on
% hold on;
% 
% 
% 
% theta = cInvKinePseudoJacobianQ4(Youbot,eeCoords, theta, 0.1, [1 1 1 1 1 0]);
% 
% hold off;


%%
% This function calculates the inverse-kinematic solution for a given
% end-effector position using the Pseudo inverse of the Jacobian Matrix
% Arguments:
% robot         -   object of SerialLink type describing the physical 
%                   configuration of the manipulator
% eeCords       -   desired end-effector coordinates
% theta         -   initial values for joint angles
% tol           -   tolerance value to stop at
% poseMask      -   6x1 vector defining the cartesian DoF of the end-effector
function theta = cInvKinePseudoJacobianQ4(robot, eeCoords, theta, tol, poseMask)

   
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
    
    count = 1;
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
               z = subIndex(robot.A(1:j-1, theta),1:3,3);
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
        
%         robot.plot(theta);
        
%         count = count +1;
%         if count == 100 || eePosOffset < tol 
%             break;
%         end

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
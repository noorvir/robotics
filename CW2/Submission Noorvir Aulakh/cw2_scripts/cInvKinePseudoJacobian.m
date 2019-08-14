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
function theta = cInvKinePseudoJacobian(robot, eeCoords, theta, tol, poseMask, use_CF_q4)

   
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

        % Use the analytical solution for Jacobian derived in q4.b
        if (use_CF_q4)
            
            % Robot parameters (shorthand variables)
            d1 = robot.d(1);
            d5 = robot.d(5);

            a2 = robot.a(2);
            a3 = robot.a(3);
            
            t1 = theta(1);
            t2 = theta(2);
            t3 = theta(3);
            t4 = theta(4);
            
            t4 = t4 + pi/2;
            t2 = t2 + pi/2;
            
            
            t5 = theta(5);
            
            t23 = t2+t3;
            t234 = t2+t3+t4;
            
            jlv1 = [-d5*sin(t1)*sin(t234) - a3*sin(t1)*cos(t23) - a2*sin(t1)*cos(t2); 
                    d5*cos(t1)*sin(t234) + a3*cos(t1)*cos(t23) + a2*cos(t1)*cos(t2);
                    0];
            
            jlv2 = [d5*cos(t1)*cos(t234)  - a3*cos(t1)*sin(t23) - a2*cos(t1)*sin(t2);
                    d5*sin(t1)*cos(t234)  - a3*sin(t1)*sin(t23) - a2*sin(t1)*sin(t2);
                    d5*sin(t234)          + a3*cos(t23)         + a2*cos(t2)];
            
            jlv3 = [d5*cos(t1)*cos(t234)  - a3*cos(t1)*sin(t23);
                    d5*sin(t1)*cos(t234)  - a3*sin(t1)*sin(t23);
                    d5*sin(t234)          + a3*cos(t23)];
            
            jlv4 = [d5*cos(t1)*cos(t234);
                    d5*sin(t1)*cos(t234);
                    d5*sin(t234)];
            
            jlv5 = [ 0;
                     0;
                     0];
            
            jav1 = [0;0;1];
            
            jav2 = [sin(t1);
                    -cos(t1);
                    0];
            
            jav3 = [sin(t1);
                    -cos(t1);
                    0];
            
            jav4 = [sin(t1);
                    -cos(t1);
                    0];
            
            jav5 = [cos(t1) * sin(t234);
                    sin(t1) * sin(t234);
                    -cos(t234)];
            
            jMat = [jlv1, jlv2, jlv3, jlv4, jlv5;
                    jav1, jav2, jav3, jav4, jav5];
           
        else
            
            % Calculate Jacobian
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
        end
       
        
        if ~(sum(poseMask) == 6) 
            % Only keep the dimensions we care about
            pose = pose(poseMask == 1);
            jMat = jMat(poseMask == 1, :);
        end

        % Compute inverse-kinematics for target ee - position
        theta = theta + (pinv(jMat) * (dPose - pose))';

        cTranslEE = subIndex(robot.fkine(theta), 1:3, 4);
        eePosOffset = norm(cTranslEE - dTranslEE); 
        
%         robot.plot(theta);
        
        count = count +1;
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
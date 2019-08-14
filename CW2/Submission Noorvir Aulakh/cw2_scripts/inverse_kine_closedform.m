function theta = inverse_kine_closedform(desired_traj, robot, option)
    
    fprintf('Calculating inverse kinematic soltion: Closed Form Method...\n');
    
    % Set whether to limit angles to maximum range of manipulator
    limit_angles = true;

    [M, N] = size(desired_traj);
    if (strcmp(option, 'q4'))
        
        theta = zeros(M, 5);
        
         % Loop through all trajectory points
        for tjPoint = 1: M
            pose = desired_traj(tjPoint,:);
            theta(tjPoint,:) = cInvKineClosedForm(pose, robot, limit_angles);
        end
    elseif (strcmp(option, 'qextra'))
        theta = zeros(M, 7);
    end
end


%%
% Computes the closed form solution for a given point and orientation. Note
% that not all orientiations are possible since the robot has 5DoF
% pose          -   translation and roll, pitch and yaw of end-effector 
% robot         -   object of SerialLink type describing the manipulator 
function theta = cInvKineClosedForm(pose, robot, limit_angles)
    
    % Homogenous transformtion matrix
    rot = rodrigues(pose(4:6));
    trans = pose(1:3)';
    tHom = [rot, trans;0 0 0 1];
    
    
    
    % Initialise theta vector for joint angles
    theta = zeros(1, 5);
    
    % Robot parameters (shorthand variables)
    d1 = robot.d(1);
    d5 = robot.d(5);
    
    a2 = robot.a(2);
    a3 = robot.a(3);
    
    % Set angles values from closed form solution
    % After setting each angle, check if it lies within the joint range for
    % the manipulator. Wrap to [-pi, pi] if needed. 
    t1 = atan2(tHom(2,4),tHom(1,4));
    [t1,ft1] = checkJointLimits(t1, 1);
    
    t234 = atan2(sqrt(tHom(3,1)^2 + tHom(3,2)^2),-tHom(3,3));
    
    % Temporary variables
    R1 = tHom(2,4)/sin(t1) - d5 * sin(t234);
    R2 = tHom(3,4) + d5 * cos(t234) - d1;

    % acos gives two possible values only one of which is correct and takes
    % us to the closest solution 
    t3 = -acos((R1^2 + R2^2 - (a2^2 + a3^2))/(2*a2*a3));
    [t3,ft3] = checkJointLimits(t3, 3);
    
    % More Temporary variables
    A = a3 * cos(t3) + a2;
    B = a3 * sin(t3);
    
    t2 = atan2(R2,R1) - atan2(B,A);
    
    t4 = t234 - t2 - t3;
    
    % Account for the offset in t2 and t4 - this was ignored earlier in
    % order to simplify notation/calculation
    t4 = t4 - pi/2;
    [t4,ft4] = checkJointLimits(t4, 4);
    
    t2 = t2 - pi/2;
    [t2,ft2] = checkJointLimits(t2, 2);
    
    % Calculate t5 in the end. This makes sure that t5 does not affect the
    % joint angle values needed to get the translational position right.
    % i.e. we are sacrificing orientational accuracy for translational
    % accuracy. This assumption might not always be true.
    t5 = atan2(-tHom(3,2)/sin(t234),tHom(3,1)/sin(t234));
    [t5,ft5] = checkJointLimits(t5, 5);
    
    thetaCF = [t1, t2, t3, t4, t5];
    theta = [t1, t2, t3, t4, t5];
    
    % If limit_angles flag is set, maximum joint angle value will be
    % limited to manipulator range
    if limit_angles
        theta = [ft1, ft2, ft3, ft4, ft5];

        % Display warning if joint angle had to be limited
        for j = 1:5
            if thetaCF(j) ~= theta(j)
                fprintf('WARNING: Joint angle limit exceeded for joint %d . Angle forcefully limited to %3.3f . \n', j, theta(j));
            end
        end
    end
end


% Checks is the input angle lies within the permissable range for the
% manipulator angles. Returns two variables:
% nTheta    -   new wrapped theta
% fTheta    -   if the wrapped theta is still outside permissable range
%               keep using original (wrapped theta for further calculations
%               but set the final value to th max joint limit
%               eg. if permissible joint value is in the range -150 < t < 150
%               and t from closed form solution is 170 then set nTheta =
%               170 but fTheta = 150.
function [nTheta, fTheta] = checkJointLimits(theta, jt)
    
    jRange = [deg2rad(-169) deg2rad(169);
              deg2rad(-65) deg2rad(90);
              deg2rad(-151) deg2rad(146);
              deg2rad(-102.5) deg2rad(102.5);
              deg2rad(-167.5) deg2rad(167.5);];
              
    nTheta = theta;
    fTheta = theta;
    
    if theta < jRange(jt,1) || theta > jRange(jt,2)

        % wrap theta to [-pi,pi]
        nTheta = wrapToPi(theta);

        % check if theta is still outside permissible bounds
        if nTheta > jRange(jt,2)
            fTheta = jRange(jt,2); 

        elseif nTheta < jRange(jt,1)
            fTheta = jRange(jt,1);
        end

    end 
end
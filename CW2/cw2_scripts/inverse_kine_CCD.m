%%
function theta = inverse_kine_CCD(desired_traj, robot)  
    
    % Tolerance - quit cInvKineCCD when norm of joint angles does not
    % change by more than this value
    tol = 0.00001;
    [M, N] = size(desired_traj);
    
    % Homogenous coordinates of joints
    theta = zeros(M,robot.n);
    eePosGlob = robot.fkine(theta(1,:));
    
    for tjPoint = 1: M
        eePosGlobD = transl(desired_traj(tjPoint,1), desired_traj(tjPoint,2), 0);
        theta(tjPoint,:) = cInvKineCCD(eePosGlob, eePosGlobD, robot, tol);
        eePosGlob = robot.fkine(theta(tjPoint,:));
    end
end


%%
% This function computes the CCD inverse kinematics for a desired 
% end-effector postion given the current joint angles.
%
% eePosGlob     - end-effector position in the global frame
% eePosGlobD    - desired end-effector position in the global frame
% robot         - 
% tol           - 
function theta = cInvKineCCD(eePosGlob, eePosGlobD, robot, tol)
    
    % Array of joint angles
    theta = zeros(1, robot.n);
    thetaP = ones(1, robot.n);
    
    % Repeat CCD until tolerance is met and the 
    while true
       
        if (abs(norm(theta) - norm(thetaP))) < tol
            break
        end
        
        % Iterate over all joints
        for j = robot.n: -1: 1
            
            % Current Joint coordinates
            cJointCoord = robot.A(1:(j-1), theta);
            % Coordinates of the end-effector in the local frame
            eePosLocal = coordTransform(eePosGlob, cJointCoord);
            % Coordinates of the desired position in the local frame
            eePosLocalD = coordTransform(eePosGlobD, cJointCoord);
            
            % Compute the siged angle between the current and desired 
            % end-effector position  
            dTheta = atan2(eePosLocal(1,4), eePosLocal(2,4)) - ...
                        atan2(eePosLocalD(1,4), eePosLocalD(2,4));
            
            if dTheta < 0
                dTheta = dTheta + 2*pi;
            end
            
            % Store previous theta values to check stop-condition
            thetaP = theta;
            % Update the joint angle
            theta(j) = theta(j) + dTheta;
            
            % Find coordinates of end effector
            eePosGlob = robot.fkine(theta);
        end
    end
end


%%
% Computes the homogenous coordinates of a frame in the the homGlobFrameCoord
% (local) frame given homGlobPointCoord  (coordinated of the frame in the
% global coordinate system)
function homLocalPointCoord = coordTransform(homGlobPointCoord, homGlobFrameCoord)
    
    homLocalPointCoord = homGlobFrameCoord \ homGlobPointCoord;
end


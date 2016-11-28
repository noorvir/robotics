
% Anonymous function to index matrices returned through function calls
global indexMat;
indexMat = @(mat,i,j) mat(i,j);


%Specify the length
L = 1;

%Specify DH parameters each link of 4R-planar manipulator
L1 = Link([0, 0, L, 0, 0], 'standard');
L2 = Link([0, 0, L, 0, 0], 'standard');
L3 = Link([0, 0, L, 0, 0], 'standard');
L4 = Link([0, 0, L, 0, 0], 'standard');

%Construct the robot
robot = SerialLink([L1, L2, L3, L4]);



theta = [0,0,0,0];
eePosGlobal = transl(4,0,0);
eePosGlobalD = transl(4,4,0);

robot.plot(theta);
cInvKineCCD(eePosGlobal, eePosGlobalD, robot, 0.0001); 


%%
% function theta = inverse_kine_CCD(desired_traj, robot)
%     [M, N] = size(desired_traj);
%     
%     % Homogenous coordinates of joints
%     jointPos = zeros(4,4,robot.n);
%     
%     % - for every end-effector position,call cInvCCD to get the joint angles 
%     
%     % - use these joint angles as initial conditions for CCD at the next
%     % end-effector position
%     theta = zeros(M, 4);
% end

%%
% This function computes the CCD inverse kinematics for a desired 
% end-effector postion given the current joint angles.
% eePosGlobD    - desired end-effector position in the global frame
% eePosGlob     - end-effector position in the global frame
% robot         - 
% tol           - 
function theta = cInvKineCCD(eePosGlob, eePosGlobD, robot, tol)
    
    global indexMat;
    
    % Desired end-effector angle from the robot origin wrt the global
    % x-axis
    theta_ee_d = acos(dot(eePosGlobD(1:3,4),[1 0 0])...
                                /(norm(eePosGlobD(1:3,4))*norm([1 0 0])));
    % Current end-effector andgle from the robot origin wrt the global 
    % x-axis
    theta_ee = acos(dot(eePosGlob(1:3,4),[1 0 0])...
                                /(norm(eePosGlob(1:3,4))*norm([1 0 0])));
    % Array of joint angles
    theta = zeros(1, robot.n);
    thetaP = ones(1, robot.n);
    % work-space radius of manipulator
    mLen = norm(indexMat(robot.fkine(theta),1:3,4));
    
    % Repeat CCD until tolerance is met and the 
    while true
       
        if (abs(norm(theta) - norm(thetaP))) < tol
            break
        end
%         if (theta_ee_d - theta_ee) < tol && ...
%                 norm(eePosGlob(1:3,4)) == mLen 
%             break;
%         elseif (theta_ee_d - theta_ee) < tol && ...
%                 norm(eePosGlobD(1:3,4)) <= mLen 
%             break;
%         end
        
        % Iterate over all joints
        for j = robot.n: -1: 1
            
            % Current Joint coordinates
            cJointCoord = robot.A(1:(j-1), theta);
            % Coordinates of the end-effector in the local frame
            eePosLocal = coordTransform(eePosGlob, cJointCoord);
            % Coordinates of the desired position in the local frame
            eePosLocalD = coordTransform(eePosGlobD, cJointCoord);

            % New current joint angle
%             theta(j) = acos(dot(eePosLocal(1:3,4),eePosLocalD(1:3,4))...
%                            /(norm(eePosLocal(1:3,4))*norm(eePosLocalD(1:3,4))));
            
            dTheta = atan2(eePosLocal(1,4), eePosLocal(2,4)) - ...
                        atan2(eePosLocalD(1,4), eePosLocalD(2,4));
            
            if dTheta < 0
                dTheta = dTheta + 2*pi;
            end
            
            thetaP = theta;
            theta(j) = theta(j) + dTheta;
            
            % Find coordinates of end effector
            eePosGlob = robot.fkine(theta);
            % Update angle of end-effector from the robot origin
            theta_ee = acos(dot(eePosGlob(1:3,4),[1 0 0])...
                                    /(norm(eePosGlob(1:3,4))*norm([1 0 0])));
%             robot.plot(theta);
        end
    end
end


%%
% Computes the homogenous coordinates of a frame in the the homGlobFrameCoord
% (local) frame given homGlobPointCoord  (coordinated of the frame in the
% global coordinate system)
function homLocalPointCoord = coordTransform(homGlobPointCoord, homGlobFrameCoord)
    
    homLocalPointCoord = inv(homGlobFrameCoord) * homGlobPointCoord;
end


function theta = inverse_kine_closedform(desired_traj, robot, option)
    [M, N] = size(desired_traj);
    if (strcmp(option, 'q4'))
        
        theta = zeros(M, 5);
        
         % Loop through all trajectory points
        for tjPoint = 1: M
            pose = desired_traj(tjPoint,:);
            
            % Convert pose into homogenous transformation matrix
            rot = rodrigues(pose(4:6));
            eeCoords = [rot, pose(1:3)'; [0 0 0 1]];

            theta(tjPoint,:) = cInvKineClosedForm(pose, robot);
        end
    elseif (strcmp(option, 'qextra'))
        theta = zeros(M, 7);
    end
end


% 
% close all;
% %Specify DH parameters each link of 4R-planar manipulator
% L1 = Link('d', 0.147, 'a', 0, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
% L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
% L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
% % L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(180)]);
% L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
% L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);
% 
% %Construct the robot
% Youbot = SerialLink([L1, L2, L3, L4, L5]);
% 
% pose = [-0.00520132587701984	-0.313479814111016	0.260615515263846	1.02399481943110	1.37948276309556	-1.47867673205012];
% % pose = [-0.127772293777214,0.0244587710530655,0.442124905491900,-0.772270009036651,-0.0278390270273121,2.59682839403522];
% theta = cInvKineClosedForm(pose, Youbot);
% 
% figure;
% hold on;
% % Youbot.plot(theta);
% Youbot.teach(theta);
% 
% plt = pose(1:3);
% plot3(plt(1),plt(2),plt(3), 'b*')
% % plot3( -0.127772293777214,0.0244587710530655,0.442124905491900, 'b*')
% xlabel('x')
% ylabel('y')
% grid on
% hold off;
% T = Youbot.fkine(theta)
%  tr2rpy(T)
% 
% 
% 
% % rot = rpy2tr(pose(4:6));
% rot = rodrigues(pose(4:6));
% trans = pose(1:3)';
% tHom = [rot, trans;0 0 0 1];
% 
% % ikineAngles = Youbot.ikine(tHom, [0,0,0,0,0],[1,1,1,1,1,0], 'ilimit', 5000) 
% 
% T;
% 
% t = rodrigues([plt(1),plt(2),plt(3)]);
% figure
% trplot(t, 'rviz', 'frame', 'Return')
% 
% 


%%
% Computes the closed form solution for a given point and orientation. Note
% the not all orientiations are possible since the robot has 5DoF
% pose          -   translation and roll, pitch and yaw of end-effector 
% robot         -   object of SerialLink type describing the manipulator 
function theta = cInvKineClosedForm(pose, robot)
    
    % Homogenous transformtion matrix
    rot = rodrigues(pose(4:6));
    trans = pose(1:3)';
    tHom = [rot, trans;0 0 0 1];
    
    
    
    % Initialise theta vector for joint angles
    theta = zeros(1, 5);
    
    d1 = robot.d(1);
    
    
    d5 = robot.d(5);
    
    a2 = robot.a(2);
    a3 = robot.a(3);
    
    % Set values from closed form solution
    t1 = atan2(tHom(2,4),tHom(1,4));
    
    t234 = atan2(sqrt(tHom(3,1)^2 + tHom(3,2)^2),-tHom(3,3));
    
    R1 = tHom(2,4)/sin(t1) - d5 * sin(t234);
    R2 = tHom(3,4) + d5 * cos(t234) - d1;
    
    
    t3 = -acos((R1^2 + R2^2 - (a2^2 + a3^2))/(2*a2*a3));
    
    A = a3 * cos(t3) + a2;
    B = a3 * sin(t3);
    
    t2 = atan2(R2,R1) - atan2(B,A);
    
    t4 = t234 - t2 - t3;
    
    t4 = t4 - pi/2;
    t2 = t2 - pi/2;
    
    t5 = atan2(-tHom(3,2)/sin(t234),tHom(3,1)/sin(t234));
%     t1 = checkJointLimits(t1, 1);
%     
%     t5 = atan2(-tHom(3,2),tHom(3,1));
%     t5 = checkJointLimits(t5, 5);
%     
%     t3 = acos((sin(t1)^2 * cos(t5)^2 * (tHom(3,4) - d5*tHom(3,3) - d1)^2 + (cos(t5) * tHom(2,4) - d5*sin(t1)*tHom(3,1))^2)/(2*a2*a3 * sin(t1)^2 * cos(t5)^2)- (a3^2 + a2^2)/(2*a2*a3));
%     % decide on the sign of t3 based on whatever gives you the correct
%     % orientation of the end-effector
%     t3 = t3;
%     t3 = checkJointLimits(t3, 3);
%     
%     R3 = tHom(3,4) - d5 * tHom(3,3)- d1;
%     R2 = (cos(t5) * tHom(2,4) - d5 * sin(t1) * tHom(3,1))/(sin(t1)*cos(t5));
%     
%     A = a3 * cos(t3) + a2;
%     B = a3 * sin(t3);
%     
%     t2 = atan2(R3, R2) - atan2(B, A);
%     t2 = checkJointLimits(t2, 2);
%     
%     t4 = atan2(tHom(2,3), -sin(t1)*tHom(3,3)) - t2 - t3;
%     
%     t4 = (t4 - pi/2);
%     t4 = checkJointLimits(t4, 4);
%     
%     t2 = (t2 - pi/2);
%     
    theta = [t1, t2, t3, t4, t5]
end

function nTheta = checkJointLimits(theta, jNum)
    
    jRange = [deg2rad(-169) deg2rad(169);
              deg2rad(-65) deg2rad(90);
              deg2rad(-151) deg2rad(146);
              deg2rad(-102.5) deg2rad(102.5);
              deg2rad(-167.5) deg2rad(167.5);];
              
    nTheta = theta;
    
    
    % If theta is outside the positive angle range for the joint, try and
    % achieve position by reversing the sign and vice-versa. 
    
    % Check if changing theta from clock-wise to anti-clockwise (or
    % vice-versa will make it stay within joint limits
    if (theta/abs(theta) * (abs(theta) - 2*pi)) < jRange(jNum, 2) && ...
        (theta/abs(theta) * (abs(theta) - 2*pi)) > jRange(jNum, 1)
        
        % If theta exceeds the positive joint bound, consider anticlockwise 
        % rotation (and vice-versa)
        if (theta < jRange(jNum, 1) || theta > jRange(jNum, 2)) 
            nTheta = theta/abs(theta) * (abs(theta) - 2*pi);
        end
    end
    
end



% -if point lies outside the range of theta1, chose the next closest point (
%   max/min angle
% -if an angle exeeds the max joint limit and reversing the direction will
%   make it work, then reverse the direction 
% -if if exeeds max limits on both tires, then choose the one that takes it
% the closest -------- you might have to sacrifice the accuracy of your end effector orientation 




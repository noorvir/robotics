
%% 
% This function uses the ikine function from Peter Corke's Robotics toolbox
% to computer the inverse kinematic solution to the Youbot
function theta = inverse_kine_PCRT(desired_traj, robot,alpha, tol, option)

[M,N] = size(desired_traj);

theta = zeros(M, robot.n);

if N == 2
    mask = [1 1 1 0 0 0 ];
    qInit = [0,0,0,0];
else
    mask = [1 1 1 1 1 0];
    qInit = [0,0,0,0,0];
end

% Calculate the workspace of the robot
ee_rangeT = robot.fkine(theta(1,:));
ee_range = norm(ee_rangeT(1:3,4));

tolO = tol;
for i = 1:size(desired_traj,1)
      
    tol = tolO;
    
    if (N == 2)
        trans = [desired_traj(i,:) 0]';
        rot= rotx(0);
        
        % Desired extension of manipulator
        ee_desiredLen = norm(desired_traj(i,1:2));
    else
        trans = desired_traj(i,1:3)';
        rot = rodrigues(desired_traj(i,4:6));
        
        % Desired extension of manipulator
        ee_desiredLen = norm(desired_traj(i,1:3));
    end
    
    
    if (ee_desiredLen > ee_range)
        % Set tolerance a percentage of minimum error possible.
        tol = abs(ee_desiredLen - ee_range) + tol * (ee_desiredLen - ee_range);
    else
        tol = tol * ee_range; 
    end
    
    T = [rot, trans; 0 0 0 1];
    
    theta(i,:) = robot.ikine(T, qInit, mask, 'alpha', alpha, 'tol', tol);
    qInit = theta(i,:);
end
    

end
T_base = eye(4);
trplot(T_base, 'rviz', 'frame', 'YB');
hold on
trplot(T_w_yba, 'rviz', 'frame', 'world Base');
plot3(translation_profile(:,1),translation_profile(:,2),translation_profile(:,3))
% robot.plot(joint_profile, 'trail', 'r-','delay', 0.009)

% trplot([rotx(-pi/2)* rotz(pi/2) ,[0.1;0.1;0];[0,0,0,1]], 'rviz', 'frame', 'EE');
hold off;


% R = rotx(-pi/2)* rotz(pi/2);
% T1 = [R,[0;0;1];[0,0,0,1]]
close all;

% %Specify DH parameters each link of 4R-planar manipulator
% L1 = Link('d', 0.147, 'a', 0, 'alpha', pi/2, 'offset', 0, 'standard');
% L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'standard');
% L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'offset', 0, 'standard');
% L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'standard');
% L5 = Link('d', 0.218, 'a', 0, 'alpha',0 , 'offset', 0, 'standard');
% 
% %Construct the robot
% r = SerialLink([L1, L2, L3, L4, L5])
% 
% d1 = 0.147;
% d5 = 0.218;
% a2 = 0.155;
% a3 = 0.135;
% 
% t1 = 0;
% t2 = 0 + pi/2;
% t3 = 0;
% t4 = 0 + pi/2;
% t5 = 0;
% 
% T = ...
% [cos(t1)*cos(t5)*cos(t4+t2+t3) + sin(t5)*sin(t1), -cos(t1)*sin(t5)*cos(t2+t3+t4)+sin(t1)*cos(t5),   cos(t1)*sin(t2+t3+t4),      d5*cos(t1)*sin(t2+t3+t4) + a3*cos(t1)*cos(t2+t3)+a2*cos(t1)*cos(t2);
% sin(t1)*cos(t5)*cos(t2+t3+t4)-cos(t1)*sin(t5),    -sin(t1)*sin(t5)*cos(t2+t3+t4)-cos(t1)*cos(t5),  sin(t1)*sin(t2+t3+t4),      d5*sin(t1)*sin(t2+t3+t4) + a3*sin(t1)*cos(t2+t3)+a2*sin(t1)*cos(t2);
% cos(t5)*sin(t2+t3+t4),                            -sin(t5)*sin(t2+t3+t4),                           -cos(t2+t3+t4),             -d5*cos(t2+t3+t4)+ a3*sin(t2+t3) + a2*sin(t2) + d1;
% 0,0,0,1];
% 
% 
% r.jacob0([t1,t2,t3,t4,t5])
% r.teach
% 
% T;
% 
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
% T = [-0.8249   -0.4151   -0.3838   -0.0913;
%     0.4208   -0.9042    0.0735    0.0175;
%    -0.3775   -0.1009    0.9205    0.4374;
%          0         0         0    1.0000];
%      
% t = rpy2r(-0.551410851752511,-0.0198774281345458,2.64347535753598)
% 
% % Homogenous transformtion matrix
% rot = rodrigues(pose(4:6));
% trans = pose(1:3)';
% tHom = [t, trans;0 0 0 1];
%     
% 
% theta = Youbot.ikine(tHom,[2,1,0.2,.6,1],[1,1,1,1,1,0])
% 
% T = Youbot.fkine(theta);
% 
% Youbot.teach(theta)
% hold on
% plot3(-0.0912500521904440,0.0174675124718185,0.437377970852528, 'b*')
% xlabel('x')
% ylabel('y')


%Specify the length
L = 1;

%Specify DH parameters each link of 4R-planar manipulator
L1 = Link([0, 0, L, 0, 0], 'standard');
L2 = Link([0, 0, L, 0, 0], 'standard');
L3 = Link([0, 0, L, 0, 0], 'standard');

R = SerialLink([L1,L2,L3]);

R

l1= 1;
l2 = 2;
l3 =1;

t1 = pi;

t2 = 100*pi;

t3 = pi/4;

J = [ -l1 * sin(t1) - l2*sin(t1+t2) - l3 *sin(t1+t2+t3),- l2*sin(t1+t2) - l3 *sin(t1+t2+t3), - l3 *sin(t1+t2+t3);
       l1 * cos(t1) + l2*cos(t1+t2) + l3 *cos(t1+t2+t3),  l2*cos(t1+t2) + l3 *cos(t1+t2+t3),   l3 *cos(t1+t2+t3);
       
       1                                               , 1                                 ,    1];

   
   
det (J)

condJ = cond(J)












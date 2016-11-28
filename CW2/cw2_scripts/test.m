close all;

%Specify DH parameters each link of 4R-planar manipulator
L1 = Link([0, 0, L, 0, 0], 'standard');
L2 = Link([0, 0, L, 0, 0], 'standard');
L3 = Link([0, 0, L, 0, 0], 'standard');
L4 = Link([0, 0, L, 0, 0], 'standard');

%Construct the robot
r = SerialLink([L1, L2, L3, L4])
r.teach

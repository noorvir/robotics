% clc;
close all;

addpath('./CW3');
addpath('./RemoteAPI');
addpath('./YouBot');

global SCENE

SCENE = 'cw3altscene.ttt';


% Specify names of object ands obstacles in struct
scene.obs = {'Cuboid0','Cuboid','Cuboid3'};
scene.obj = {'Rectangle14','Cuboid2','Rectangle15'};
% Youbot Joints
scene.youbot = {'youBotArmJoint0', 'youBotArmJoint1', 'youBotArmJoint2', ...
                'youBotArmJoint3', 'youBotArmJoint4'};
scene.stack = {'Rectangle13'};
% End-effector
scene.EE = {'youBotGripperJoint1'};

%Specify DH parameters each link of 4R-planar manipulator
L1 = Link('d', 0.147, 'a', 0.033, 'alpha', pi/2, 'qlim', [deg2rad(-169) deg2rad(169)]);
L2 = Link('d', 0, 'a', 0.155, 'alpha', 0, 'offset', pi/2, 'qlim', [deg2rad(-65) deg2rad(90)]);
L3 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'qlim', [deg2rad(-151) deg2rad(146)]);
L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-102.5) deg2rad(102.5)]);
L5 = Link('d', 0.218, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-167.5) deg2rad(167.5)]);

%Construct the robot
Youbot = SerialLink([L1, L2, L3, L4, L5]);

global vrep
global clientID
% Open connection to vrep remote API to get scene information
vrep = remApi('remoteApi');
vrep.simxFinish(-1); % close remote simulation just-in-case
clientID = vrep.simxStart('127.0.0.1', 19997, true, true, 5000, 5);
% Load the vrep scene file
res = vrep.simxLoadScene(clientID,SCENE,1,vrep.simx_opmode_blocking);
if res~=0
    fprintf('loading scene failed. Try again')
    error('LOAD ERROR');
end

% Get handles for all objects now and save them because this is time
% consuming to do in real-time
fprintf('Getting VREP handles for items in the scene. \nThis may or may not take some time...\n');
% handles = get_scene_data(scene, clientID, vrep, 'getHandles', []);
% save('handles.mat','handles');
handles = load('handles.mat');
handles = handles.handles;
fprintf('Done\n');
% fprintf('Getting scene data');

% Sometimes VREP doesn't return any data - here we test using one value 
% randomly and keep trying until it does
% while true
%     % get_scene_data
%     [data] = get_scene_data(scene, clientID, vrep, 'getPosOr', handles);
%     
%     if data.obj{1}.position(1) ~= 0
%         break
%     end
% end

% data = load('data.mat');
% data = data.data;

% tr = [-0.1132   -0.9606   -2.0705    1.4603   0];

% t = Youbot.fkine([0,0,0,pi/2,0]);
% 
% tr = inverse_kinematics(t,Youbot);

% joint_data = ones(size(joint_data,1),5);
% joint_data(:,1) = tr(1) * joint_data(:,1);
% joint_data(:,2) = tr(2) * joint_data(:,2);
% joint_data(:,3) = tr(3) * joint_data(:,3);
% joint_data(:,4) = tr(4) * joint_data(:,4);
% joint_data(:,5) = tr(5) * joint_data(:,5);


fprintf('Starting simulation.\n');



% LGripper = [linspace(0, 0.02, size(joint_data,1))'; linspace(0.02, 0, size(joint_data,1))'];
% RGripper = [linspace(0, -0.02, size(joint_data,1))'; linspace(-0.02, 0, size(joint_data,1))'];
% GripperData = [LGripper, RGripper];
% GripperData = repmat(GripperData, [4, 1]);
% GripperData = GripperData(1:length(joint_data), :);
% joint_data = [joint_data, GripperData];
% jointData.joint_profile(1:end, 1:5) = 0; %We set this to zeros just to see the gripper motion.
% JPM = joint_data;
% clear jointData;

CW3Setup
Trajectory(scene, clientID, vrep, handles, Youbot, setPoseArmPub, closeGripperPub, openGripperPub);

CW3FreeResources

% plot
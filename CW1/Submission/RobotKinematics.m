% Example script to set up robot from DH parameters in robotics toolbox
% Author: George Dwyer
clear
close all
% Each row of the DH table, is represented by the link class.
% The link is constructed using key value pairs and flags.
% Key Value Pairs:
%   theta - Joint angle (not specified if revolute joint is used)(default)
%   d - link offset (not specified if prismatic joint is used)
%   a - link length
%   alpha - link twist
%   offset - joint variable offset
%   qlim - joint limits (radians if revolute)

% Flags:
%   revolute - indicates revolute joint
%   prismatic - indicates prismatic joint
%   standard - using standard dh notation
%   modified - using modified dh notation


L1 = Link('d', 0.147, 'a', 0, 'alpha', -pi/2, 'qlim', [deg2rad(169) deg2rad(169)], 'standard');
L2 = Link('d', 0, 'a', 0.155, 'alpha', 0,'offset', -pi/2, 'qlim', [deg2rad(90) deg2rad(-65)], 'standard');
L3 = Link('d', 0, 'a', 0.135, 'alpha', 0,  'qlim', [deg2rad(-146) deg2rad(-50)], 'standard');
L4 = Link('d', 0, 'a', 0, 'alpha', pi/2,'offset', pi/2, 'qlim', [deg2rad(102.5) deg2rad(-102.5)], 'standard');
L5 = Link('d', 218, 'a', 0, 'alpha', pi/2, 'offset', pi/2, 'qlim', [deg2rad(-167.5) deg2rad(-167.5)], 'standard');

% These links may then be assembled into a manipulator using the SerialLink
% class. Each link must be defined using the same dh convention.
R = SerialLink([L1 L2 L3 L4 L5], 'name', 'Example Manipulator');

R.teach
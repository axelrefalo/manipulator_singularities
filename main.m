clear all; % erase all the remaining data
close all; % close the windows

% change the path of the directory
dir_path = '/Users/axel/Desktop/Axel/ETS/CoRo/Matlab/manipulator singularities/';
addpath(fullfile(dir_path, 'function'));

syms tet1 tet2 tet3 tet4 tet5 tet6 real

% the orgin of the needs to be (0, 0, 0) and the first axis (0, 0, 1)
DH_table = [
    tet1,              0.265,  0,      -pi/2;
    tet2 - pi/2,       0,      0.444,   0;
    tet3,              0,      0.110,  -pi/2;
    tet4,              0.470,  0,       pi/2;
    tet5,              0,      0.080,  -pi/2;
    tet6 + pi,         0.101,  0,       0
];

urdf_path = fullfile(dir_path, 'robot/crb15000_5_95/urdf/crb15000_5_95.urdf');

J = searchSingularities(urdf_path, DH_table);
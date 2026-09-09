%% initials of the PID simulink files
% run PID_init
% tune gains
% run save_PID_gains
% run this script to test.
clear; clc; close all;
g = 9.81;
dt = 0.01;
tf = 50;
Ix = 0.3;   % moment of inertia (kg*m^2)
Iy = 0.4;   % moment of inertia (kg*m^2)
Iz = 0.5;   % moment of inertia (kg*m^2)
m = 1;
A_pos = zeros(6,6);

B_pos = [1, 0, 0, 0, 0, 0; 
        0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0
        0, 0, 0, -1, 0, 0;
        0, 0, 0, 0, -1, 0;
        0, 0, 0, 0, 0, -1];

C_pos = [1, 0, 0, 0, 0, 0; 
        0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0
        0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 1];

D_pos = zeros(6,6);

A_att = zeros(6,6);

B_att = [1   0   0   0   0   0 
         0   1   0   0   0   0
         0   0   1   0   0   0
         0   0   0   1/Ix  0     0
         0   0   0   0    1/Iy   0
         0   0   0   0     0     1/Iz];

C_att = [1, 0, 0, 0, 0, 0; 
        0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0
        0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 1];

D_att = zeros(6,6);
clc; clear; close all;
% my pc path
% addpath("/home/engelhardt/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions/")

% lab work stations paths
% addpath("/home/users10/re606359/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions")

addpath("C:\Users\re606359\Desktop\Advanced-Systems-Labs-\Infinite_SNAC_2023\functions " )

% Define plant dynamics
Ix = 2;   % moment of inertia (kg*m^2)
Iy = 2;   % moment of inertia (kg*m^2)
Iz = 4;   % moment of inertia (kg*m^2)
m = 3.2; 

N_states = 12;
N_patterns = 1000;
max_training_loop = 2000;
threshold = 1e-5;
dt = 0.004;
discount = 0.99;
Attitude_Q = diag([100,100,100,100,100,100,100,100,100,100,100,100,100])*10000;
Attitude_R = diag([1,1,1,1])*100;

% training domain for all 12 states
X_max = 1000; X_min = -1000;
Y_max = 1000; Y_min = -1000;
Z_max = 100; Z_min = -100;

u_max = 20; u_min = -20;
v_max = 20; v_min = -20;
w_max = 20; w_min = -20;

PHI_max = pi/6; PHI_min = -pi/6;
THE_max = pi/6; THE_min = -pi/6;
PSI_max = pi; PSI_min = -pi;

p_max =  pi/8; p_min =  -pi/8;
q_max =  pi/8; q_min =  -pi/8;
r_max =  pi/10; r_min =  -pi/10;

% Partial x_k+1 / partial x_k grad(f(x) + g(x) *u)
F1_grad = @(x)
A = @(x)...
    [
    1,          0,          0,            dt,        0,           0,              0,          0,              0,
    0,          1,          0,            0,         dt,          0; 
    0,          0,          1,            0,         0,           dt; 
    0,          0,          0,            1,         0,           0; 
    0,          0,          0,            0,         1,           0; 
    0,          0,          0,            0,         0,           1; 
    ]; % row representation
clear; clc; close all;

%% create global structs (all controllers will use this)
Q_pos = diag([1,1,1,1,1,1])*1; % continous time parameters (PLEASE CHECK IF YOUR WORKING IN DISCRETE OR CONT)
R_pos = diag([1,1,1])*1;
Q_att = diag([1,1,1,1,1,1])*1;
R_att = diag([1,1,1])*1;

dt = 0.01;
tf = 50;
g = 9.81;
m = 1; 
Ix = 0.3;   Iy = 0.4;   Iz = 0.5;
T = 0:dt:tf-dt;
N = length(T);
W_nominal = 5; % wind speed for wind simulations (m/s)

global_parameters.dt        = dt;    % time step
global_parameters.tf        = tf;       % final time
global_parameters.g         = g;     % gravity (m/s^2)
global_parameters.m         = m;        % mass (kg)
global_parameters.Ix        = Ix;      % moments of inertia (kg*m^2)
global_parameters.Iy        = Iy;      %
global_parameters.Iz        = Iz;      %
global_parameters.r_yaw     = pi/3*ones(1,length(T)); % reference yaw
global_parameters.W_nominal = W_nominal;

%% reference for all controllers
ref = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t];

%% simualtion parameters
state_noise = 0.00;             % percent noise as a decimal
control_noise = 0.00;           % percent noise as a decimal
saturation_logic = 0;           % logical value to select saturation mode 0 = no, 1 = yes
wind_logic = 0;                 % logical value to select wind noise mode (the same wind dist will be used in every case) 0 = no, 1 = yes
reference_smoothing_logic = 0;  % logic for reference smoothing: 0 = no, 1 = yes
payload_uncert_logic      = 0;
payload_uncert = [.1;.05;.05;.05];

sim_params.state_noise = state_noise;
sim_params.control_noise = control_noise;
sim_params.saturation_logic = saturation_logic;
sim_params.wind_logic = wind_logic; 
sim_params.reference_smoothing_logic = reference_smoothing_logic;
sim_params.payload_uncert = [payload_uncert_logic; payload_uncert];

%% Initial Condition
% IC = zeros(12,1); % easy

IC = [5; 5; 0; zeros(9,1)]; % moderate

% IC = [5; 5; 0; -1.16; -5.10; 0.79; 0.13; -0.54; pi/2; -0.071; -0.0025; 0.51]; % hard

%% global cost function stucts
global_position.Q_pos = Q_pos;
global_position.R_pos = R_pos;
global_attitude.Q_att = Q_att;
global_attitude.R_att = R_att;

% Simulate Discounted_Single_Critic (need do) (comment out to speed up the code ALOT)
DCS_ref1_safe = simulate_Discounted_Single_Critic(global_position, global_attitude, global_parameters,ref, IC, sim_params,1);
DCS_ref1_Unsafe = simulate_Discounted_Single_Critic(global_position, global_attitude, global_parameters,ref, IC, sim_params,0);

ref = @(t) [5*cos(0.2*t)+2.5;       % reference_x
             5*sin(0.2*t)+2.5;     % reference_y
              -2.5];      % reference_z

DCS_ref2_safe = simulate_Discounted_Single_Critic(global_position, global_attitude, global_parameters,ref, IC, sim_params,1);
DCS_ref2_Unsafe = simulate_Discounted_Single_Critic(global_position, global_attitude, global_parameters,ref, IC, sim_params,0);

RMSE_error = sqrt( mean( sum(DCS_ref1_safe.error(1:3,:).^2,1)))
max_tracking_error = max(sqrt(sum(DCS_ref1_safe.error(1:3,:).^2,1)))
h_min = min(DCS_ref1_safe.h_log)

RMSE_error_ref2 = sqrt( mean( sum(DCS_ref2_safe.error(1:3,:).^2,1)))
max_tracking_error_ref2 = max(sqrt(sum(DCS_ref2_safe.error(1:3,:).^2,1)))
h_min_ref2 = min(DCS_ref2_safe.h_log)
save("Drone_shielding.mat")

% disp("======= Discounted_Single_Critic Results ======")
% % fprintf("Discounted_Single_Critic simulation time: %.4f\n", DCS_ref1_safe.simtime);
% % fprintf("Discounted_Single_Critic total tracking error: %.4f\n", DCS_ref1_safe.Discounted_Single_Critic_tracking_error_norm);
% fprintf("Discounted_Single_Critic tracking cost: %.4f\n", DCS_ref1_safe.all_cost(2));
% fprintf("Discounted_Single_Critic control cost: %.4f\n", DCS_ref1_safe.all_cost(3));

radius = 2;
c = [-1;-1.5;-2];
nSphere = 30;                              % resolution
[Xs,Ys,Zs] = sphere(nSphere);              % unit sphere
Xs = radius*Xs + c(1);
Ys = radius*Ys + c(2);
Zs = radius*Zs + c(3);

%%ref 1
refxyz = DCS_ref1_safe.X_ref(1:6,:);
figure;
grid on
hold on
plot3(refxyz(1,:), refxyz(2,:), -refxyz(3,:), '--', 'Color', [0 1 0], 'Linewidth', 1.5)
plot3(DCS_ref1_Unsafe.X(1,:), DCS_ref1_Unsafe.X(2,:), -DCS_ref1_Unsafe.X(3,:), 'k', 'Marker', 'o', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:700:length(DCS_ref1_Unsafe.X(1,:)), 'MarkerSize', 5)
plot3(DCS_ref1_safe.X(1,:), DCS_ref1_safe.X(2,:), -DCS_ref1_safe.X(3,:), 'b', 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:500:length(DCS_ref1_safe.X(1,:)), 'MarkerSize', 5)
surf(Xs, Ys, -Zs,'FaceAlpha', 0.20,'EdgeAlpha', 0.10,'FaceColor', [1 0 0],'EdgeColor', 'k');
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
zlim([0 4])
legend(["$r_1(t)$","No-Shielding", "Shielding", "$h(x) = 0$"], 'Interpreter', 'latex');

refxyz = DCS_ref2_safe.X_ref(1:6,:);
figure;
grid on
hold on
plot3(refxyz(1,:), refxyz(2,:), -refxyz(3,:), '--', 'Color', [0 1 0], 'Linewidth', 1.5)
plot3(DCS_ref2_Unsafe.X(1,:), DCS_ref2_Unsafe.X(2,:), -DCS_ref2_Unsafe.X(3,:), 'k', 'Marker', 'o', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:700:length(DCS_ref2_Unsafe.X(1,:)), 'MarkerSize', 5)
plot3(DCS_ref2_safe.X(1,:), DCS_ref2_safe.X(2,:), -DCS_ref2_safe.X(3,:), 'b', 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:500:length(DCS_ref2_safe.X(1,:)), 'MarkerSize', 5)
surf(Xs, Ys, -Zs,'FaceAlpha', 0.20,'EdgeAlpha', 0.10,'FaceColor', [1 0 0],'EdgeColor', 'k');
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
zlim([-.1 4])
legend(["$r_2(t)$","No-Shielding", "Shielding", "$h(x) = 0$"], 'Interpreter', 'latex');

figure;
subplot(2,1,1)
hold on;
plot(T, DCS_ref1_Unsafe.h_log, 'k', 'Marker', 'o', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:700:length(DCS_ref2_Unsafe.X(1,:)), 'MarkerSize', 5);
plot(T, DCS_ref1_safe.h_log, 'b', 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:500:length(DCS_ref2_safe.X(1,:)), 'MarkerSize', 5)
yline(0,'r--');
xlabel("Time (s)", 'Interpreter','latex');
ylabel("$h(x)$",   'Interpreter','latex');
grid on;
legend("Unsafe","Safe","$h(x) = 0$", 'Interpreter','latex')
title("$r_1(t)$ Tracking Safety Constraint", 'Interpreter','latex')

subplot(2,1,2)
hold on;
plot(T, DCS_ref2_Unsafe.h_log, 'k', 'Marker', 'o', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:700:length(DCS_ref2_Unsafe.X(1,:)), 'MarkerSize', 5);
plot(T, DCS_ref2_safe.h_log, 'b', 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:500:length(DCS_ref2_safe.X(1,:)), 'MarkerSize', 5)
yline(0,'r--');
xlabel("Time (s)", 'Interpreter','latex');
ylabel("$h(x)$",   'Interpreter','latex');
grid on;
legend("Unsafe","Safe","$h(x) = 0$", 'Interpreter','latex')
title("$r_2(t)$ Tracking Safety Constraint", 'Interpreter','latex')

refxyz1 = DCS_ref1_safe.X_ref(1:6,:);
refxyz2 = DCS_ref2_safe.X_ref(1:6,:);
figure;
grid on
hold on
plot3(refxyz1(1,:), refxyz1(2,:), -refxyz1(3,:), 'Color', [0 1 0], 'Linewidth', 1.5)
plot3(refxyz2(1,:), refxyz2(2,:), -refxyz2(3,:), 'Color', [0 0 1], 'Linewidth', 1.5)
surf(Xs, Ys, -Zs,'FaceAlpha', 0.20,'EdgeAlpha', 0.10,'FaceColor', [1 0 0],'EdgeColor', 'k');
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
zlim([-.1 4])
legend(["$r_1(t)$","$r_2(t)$", "$h(x) = 0$"], 'Interpreter', 'latex');
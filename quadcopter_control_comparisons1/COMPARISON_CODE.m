clear; clc; close all;
warning('off','all')
%% note: Please follow any conventions you see in this workspace
    % -> basis vector functions or controller functions start with "a_" prefix to keep the functions folder neat
    % -> .mat files start with "z_" prefix to keep them all orgainized at the bottom
% this functions folder may contain about 50 or so files by the time everything is finished, this will save time later on 
% we can also have a folder for each controller and define a path for each, but for now this is faster
addpath('functions')

%% global cost function stucts
global_position.Q_pos = diag([1,1,1,1,1,1])*1;
global_position.R_pos = diag([1,1,1])*1;
global_attitude.Q_att = diag([1,1,1,1,1,1])*1;
global_attitude.R_att = diag([1,1,1])*1; %continous time parameters (PLEASE CHECK IF YOUR WORKING IN DISCRETE OR CONT)
    % not everything has to be trained on the same cost function but they
    % should all be graded the same. Some controllers are more senitive to
    % cost function params or have different structures all together. Try to 
    % log any different cost function params here to keep track. 

global_parameters.dt        = 0.01;    % time step
global_parameters.tf        = 50;       % final time
global_parameters.g         = 9.81;     % gravity (m/s^2)
global_parameters.m         = 1;        % mass (kg)
global_parameters.Ix        = 0.3;      % moments of inertia (kg*m^2)
global_parameters.Iy        = 0.4;      %
global_parameters.Iz        = 0.5;      %
T = 0:global_parameters.dt:global_parameters.tf-global_parameters.dt;
N = length(T);
global_parameters.r_yaw     = pi/4*ones(1,length(T)); % reference yaw

%% reference for all controllers
ref = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t];

%% simualtion parameters (For testing controller)
global_parameters.W_nominal = 5; % wind speed for wind simulations (m/s)
sim_params.state_noise = 0.1;             % percent noise as a decimal
sim_params.control_noise = .1;           % percent noise as a decimal
sim_params.saturation_logic = 1;           % logical value to select saturation mode 0 = no, 1 = yes
sim_params.wind_logic = 0;                 % logical value to select wind noise mode (the same wind dist will be used in every case) 0 = no, 1 = yes 
sim_params.reference_smoothing_logic = 0;
payload_uncert_logic      = 0;
payload_uncert = [.1;.01;.01;.01];
sim_params.payload_uncert = [payload_uncert_logic; payload_uncert];

%% Initial Condition
% IC = zeros(12,1); % eas
IC = [5; 5; 0; zeros(9,1)]; % moderate
% IC = [5; 5; 0; -1.16; -5.10; 0.79; 0.13; -0.54; pi/2; -0.071; -0.0025; 0.51]; % hard

tic
%% traditional controllers
disp("======= PID SIM ======")
pid_results = simulate_PID(global_position, global_attitude, global_parameters,ref, IC, sim_params);
disp("======= LQR SIM ======")
lqr_results = simulate_LQR(global_position, global_attitude, global_parameters,ref, IC, sim_params);
disp("======= SMC SIM ======")
SMC_results = simulate_SMC(global_position, global_attitude, global_parameters,ref, IC, sim_params);
disp("======= MPC SIM ======")
MPC_results = simulate_MPC(global_position, global_attitude, global_parameters,ref, IC, sim_params);
disp("======= FL SIM ======")
FL_results = simulate_FL(global_position, global_attitude, global_parameters,ref, IC, sim_params);
disp("======= Bakstp SIM ======")
Backstepping_results = simulate_Backstepping(global_position, global_attitude, global_parameters,ref, IC, sim_params);

%% Offline ADP methods
disp("======= Offline AC SIM ======")
Offline_AC_results = simulate_Offline_AC(global_position, global_attitude, global_parameters,ref, IC, sim_params);
disp("======= Offline SNAC SIM ======")
Offline_SNAC_results = simulate_Offline_SNAC(global_position, global_attitude, global_parameters,ref, IC, sim_params);
% disp("======= Flex SNAC SIM ======")
% Simulate Offline_Flex_SNAC (Need do) ####################### PLease finish #####################
% Flex_SNAC_results = simulate_Flex_SNAC(global_position, global_attitude, global_parameters,ref, IC, sim_params);

%% Online ADP methods (Gradient Decent)
disp("======= Online AC SIM ======")
Online_AC_results = simulate_Online_AC(global_position, global_attitude, global_parameters, ref, IC, sim_params);
disp("======= Online SNAC SIM ======")
Online_SNAC_results = simulate_Online_SNAC(global_position, global_attitude, global_parameters,ref, IC, sim_params);
disp("======= Online Single Critic SIM ======")
Discounted_Single_Critic_results = simulate_Discounted_Single_Critic(global_position, global_attitude, global_parameters, ref, IC, sim_params);

%% Online ADP methods (RLS)
disp("======= Online RLS SNAC SIM ======")
Online_SNAC_RLS_results = simulate_Online_SNAC_RLS(global_position, global_attitude, global_parameters,ref, IC, sim_params);

%% Policy Gradient methods
% disp("======= DDPG SIM ======")
% DDPG_results = simulate_DDPG(global_position, global_attitude, global_parameters,ref, IC, sim_params);
% disp("======= PPO SIM ======")
% PPO_results = simulate_PPO(global_position, global_attitude, global_parameters,ref, IC, sim_params);

total_runtime = toc;
disp("======= SIMS complete ======")
fprintf("Total run time: %.4f sec \n", total_runtime);

global_results.Offline_SNAC = Offline_SNAC_results;
global_results.Offline_AC = Offline_AC_results;
global_results.MPC = MPC_results;
global_results.LQR = lqr_results;
global_results.PID = pid_results;
global_results.SMC = SMC_results;
global_results.FL = FL_results;
global_results.Backstepping = Backstepping_results;
global_results.Online_AC = Online_AC_results;
global_results.Online_SNAC = Online_SNAC_results;
global_results.Discounted_Single_Critic = Discounted_Single_Critic_results;
global_results.Online_SNAC_RLS = Online_SNAC_RLS_results;

% global_results.DDPG = DDPG_results;
% global_results.PPO = PPO_results;

%% print stuff
disp("#############+++++++++++ Simulation Parameters++++++++++++++++++++################")
fprintf("dt = %f, noise = [%f,%f], wind?? == %f, Nominal Wind == %f\n",global_parameters.dt,sim_params.state_noise,sim_params.control_noise,sim_params.wind_logic, global_parameters.W_nominal)
[metrics, names, Tmetrics] = extract_control_metrics(global_results, global_parameters.dt);
disp(Tmetrics)
table_to_latex(Tmetrics, 'z_thesis_p1.tex');

%% Plot tracking error
figure;
hold on; box on;
set(gca, 'FontSize', 16, ...
         'TickLabelInterpreter','latex', ...
         'XGrid','on', 'YGrid','on', ...
         'GridLineStyle','--')   % dashed grid
plot(T(1:end-1), Offline_SNAC_results.Offline_SNAC_tracking_error(1,1:N-1),'-.',  'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:600:length(T), 'MarkerSize', 5);

plot(T(1:end-1), Offline_AC_results.Offline_AC_tracking_error(1,1:N-1), '-.', 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:700:length(T), 'MarkerSize', 5)

plot(T(1:end-1), MPC_results.MPC_tracking_error(1,1:N-1),  'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:900:length(T), 'MarkerSize', 5);

plot(T(1:end-1), lqr_results.lqr_tracking_error(1,1:N-1),'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:800:length(T), 'MarkerSize', 5);

plot(T(1:end-1), pid_results.pid_tracking_error(1,1:N-1), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5);
plot(T(1:end-1), SMC_results.SMC_tracking_error(1,1:N-1),":", 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1100:length(T), 'MarkerSize', 5)
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Tracking Error (m)', 'Interpreter', 'latex');
title('Tracking Error Norm', 'Interpreter', 'latex');
legend(["SNAC", "AC", "MPC", "LQR", "PID", "SMC"], 'Interpreter', 'latex');
grid on;

figure;
grid on
hold on
plot3(Offline_SNAC_results.X(1,:), Offline_SNAC_results.X(2,:), -Offline_SNAC_results.X(3,:),'-.',  'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:600:length(T), 'MarkerSize', 5);

plot3(Offline_AC_results.X(1,:), Offline_AC_results.X(2,:), -Offline_AC_results.X(3,:), '-.', 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:700:length(T), 'MarkerSize', 5)

plot3(MPC_results.X(1,:), MPC_results.X(2,:), -MPC_results.X(3,:),  'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:900:length(T), 'MarkerSize', 5);

plot3(lqr_results.X(1,:), lqr_results.X(2,:), -lqr_results.X(3,:),'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:800:length(T), 'MarkerSize', 5);

plot3(pid_results.X(1,:), pid_results.X(2,:), -pid_results.X(3,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5);
plot3(SMC_results.X(1,:), SMC_results.X(2,:), -SMC_results.X(3,:),":", 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1100:length(T), 'MarkerSize', 5)
plot3(Offline_SNAC_results.X_ref(1,:), Offline_SNAC_results.X_ref(2,:), -Offline_SNAC_results.X_ref(3,:), "g");

xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
legend([ "SNAC", "AC", "MPC", "LQR", "PID", "SMC","$r(t)$"], 'Interpreter', 'latex');

figure;
hold on;
plot(T(1:end-1), pid_results.pid_tracking_error(1,1:N-1),  'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:900:length(T), 'MarkerSize', 5);
plot(T(1:end-1), lqr_results.lqr_tracking_error(1,1:N-1), 'k', 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5);
plot(T(1:end-1), SMC_results.SMC_tracking_error(1,1:N-1), ...
    'Color', [1 0 1], 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1100:length(T), 'MarkerSize', 5)
plot(T(1:end-1), FL_results.FL_tracking_error(1,1:N-1), ...
    'Color', [0 0.6 0], 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1200:length(T), 'MarkerSize', 5);
plot(T(1:end-1), Backstepping_results.Backstepping_tracking_error(1,1:N-1), ...
    'Color', [1 0.7 0], 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1250:length(T), 'MarkerSize', 5);
plot(T(1:end-1), MPC_results.MPC_tracking_error(1,1:N-1), ...
    'Color', [0 0.6 1], 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1700:length(T), 'MarkerSize', 5);
plot(T(1:end-1), Offline_AC_results.Offline_AC_tracking_error(1,1:N-1), '-.', ...
    'Color', [0 0 1], 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1300:length(T), 'MarkerSize', 5)
plot(T(1:end-1), Offline_SNAC_results.Offline_SNAC_tracking_error(1,1:N-1),'-.',  'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1450:length(T), 'MarkerSize', 5);
% plot(T(1:end-1), Flex_SNAC_results.Flex_SNAC_tracking_error(1,1:N-1),'-.',  'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:1400:length(T), 'MarkerSize', 5);
plot(T(1:end-1), Online_AC_results.Online_AC_tracking_error(1,1:N-1), '--', ...
    'Color', [0 1 1], 'Marker', 'x', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1500:length(T), 'MarkerSize', 5)
plot(T(1:end-1), Online_SNAC_results.Online_SNAC_tracking_error(1,1:N-1), '--', ...
    'Color', [0 1 0], 'Marker', 'x', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1600:length(T), 'MarkerSize', 5)
plot(T(1:end-1), Discounted_Single_Critic_results.Discounted_Single_Critic_tracking_error(1,1:N-1), '--', ...
    'Color', [1 0 0], 'Marker', 'x', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1700:length(T), 'MarkerSize', 5)
plot(T(1:end-1), Online_SNAC_RLS_results.Online_SNAC_RLS_tracking_error(1,1:N-1), '--', ...
    'Color', [0.5 0.3 0], 'Marker', 'hexagram', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1600:length(T), 'MarkerSize', 5)
% plot(T(1:end-1), DDPG_results.DDPG_tracking_error(1,1:N-1), ':', ...
%     'Color', [0.5 0.3 1], 'Marker', "diamond", 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:1250:length(T), 'MarkerSize', 5)
% plot(T(1:end-1), PPO_results.PPO_tracking_error(1,1:N-1), ':', ...
%     'Color', [1 0.3 1], 'Marker', "diamond", 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:1150:length(T), 'MarkerSize', 5)
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% ylim([0 1])
title('L2 Norm Tracking Error', 'Interpreter', 'latex');
legend(["PID","LQR","SMC","FL","Backstep","MPC",... % classical controllers
    "AC","SNAC",... % offline
    "O-AC","O-SNAC","O-Single Critic","O-SNAC RLS"... % online control
    ], 'Interpreter', 'latex');
grid on;

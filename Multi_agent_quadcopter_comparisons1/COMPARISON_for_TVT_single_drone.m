clear; clc; close all;
addpath('functions')
warning('off','all')
%% global cost function stucts
global_position.Q_pos = diag([1,1,1,1,1,1])*1;
global_position.R_pos = diag([1,1,1])*1;
global_attitude.Q_att = diag([1,1,1,1,1,1])*1;
global_attitude.R_att = diag([1,1,1])*1; 
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
% ref = @(t)[9.81*cos(0.2*t);       
%              9.81*sin(0.2*t);    
%              -4*cos(1*t)-9.81];

ref = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t];

%% simualtion parameters (For testing controller)
global_parameters.W_nominal = 5; % wind speed for wind simulations (m/s)
sim_params.state_noise = 0.2;             % percent noise as a decimal
sim_params.control_noise = 1;           % percent noise as a decimal
sim_params.saturation_logic = 1;           % logical value to select saturation mode 0 = no, 1 = yes
sim_params.wind_logic = 1;                 % logical value to select wind noise mode (the same wind dist will be used in every case) 0 = no, 1 = yes 
sim_params.reference_smoothing_logic = 0;

%% Initial Condition
% IC = zeros(12,1); % easy
IC = [5; 5;  0; -3; 2; 0; zeros(6,1)];

tic
seed = randi(2^32 - 1);
Wxyz = gen_wind_vector(global_parameters.W_nominal,global_parameters.dt,global_parameters.tf, seed);

pid_results = simulate_PID(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
lqr_results = simulate_LQR(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
SMC_results = simulate_SMC(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
FL_results = simulate_FL(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
Backstepping_results = simulate_Backstepping(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
% % 
Offline_AC_results = simulate_Offline_AC(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
Offline_SNAC_results = simulate_Offline_SNAC(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
Flex_SNAC_results = simulate_Flex_SNAC(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
% 
MPC_results = simulate_MPC(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
% DDPG_results = simulate_DDPG(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);
% PPO_results = simulate_PPO(seed,global_position, global_attitude, global_parameters,ref, IC, sim_params, Wxyz);

%% create a stuct to make a table 
disp("======= SIMS complete ======")
% global_results.Flex_SNAC = Flex_SNAC_results;
global_results.Offline_SNAC = Offline_SNAC_results;
global_results.Offline_AC = Offline_AC_results;
global_results.LQR = lqr_results;
global_results.MPC = MPC_results;
% global_results.PPO = PPO_results;
% global_results.DDPG = DDPG_results;
% 
global_results.PID = pid_results;
global_results.SMC = SMC_results;
% global_results.FL = FL_results;
% global_results.Backstepping = Backstepping_results;



%% create a table and save as latex file
disp("#############+++++++++++ Simulation Parameters++++++++++++++++++++################")
fprintf("dt = %f, noise = [%f,%f], wind?? == %f, Nominal Wind == %f\n",global_parameters.dt,sim_params.state_noise,sim_params.control_noise,sim_params.wind_logic, global_parameters.W_nominal)
[metrics, names, Tmetrics] = extract_control_metrics(global_results, global_parameters.dt);
disp(Tmetrics)
% save("ideal_case_again_again.mat")
table_to_latex(Tmetrics, 'z_single_agent_ideal_newnewnew.tex');


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
% %% Plot tracking error
% figure;
% hold on;
% plot(T(1:end-1), Flex_SNAC_results.Flex_SNAC_tracking_error(1,1:N-1),'-.',  'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:500:length(T), 'MarkerSize', 5);
% 
% plot(T(1:end-1), Offline_SNAC_results.Offline_SNAC_tracking_error(1,1:N-1),'-.',  'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:600:length(T), 'MarkerSize', 5);
% 
% plot(T(1:end-1), Offline_AC_results.Offline_AC_tracking_error(1,1:N-1), '-.', ...
%     'Color', [0 0 1], 'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:700:length(T), 'MarkerSize', 5)
% 
% plot(T(1:end-1), MPC_results.MPC_tracking_error(1,1:N-1), ...
%     'Color', [0 0.6 1], 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:900:length(T), 'MarkerSize', 5);
% 
% plot(T(1:end-1), lqr_results.lqr_tracking_error(1,1:N-1), 'k', 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:800:length(T), 'MarkerSize', 5);
% 
% plot(T(1:end-1), pid_results.pid_tracking_error(1,1:N-1), 'b', 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5);
% plot(T(1:end-1), SMC_results.SMC_tracking_error(1,1:N-1), ...
%     'Color', [1 0 1], 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:1100:length(T), 'MarkerSize', 5)
% plot(T(1:end-1), FL_results.FL_tracking_error(1,1:N-1),'Color', [0 0.6 0], 'Marker', '*', 'LineWidth', 0.9,'MarkerIndices', 1:1200:length(T), 'MarkerSize', 5);
% plot(T(1:end-1), Backstepping_results.Backstepping_tracking_error(1,1:N-1), 'Color', [1 0.7 0], 'Marker', '*', 'LineWidth', 0.9,'MarkerIndices', 1:1300:length(T), 'MarkerSize', 5);
% 
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% % title('L2 Norm Tracking Error', 'Interpreter', 'latex');
% legend(["Flexible SNAC", "SNAC", "AC", "MPC", "LQR", "PID", "SMC", "FL", "Backstepping"
%     ], 'Interpreter', 'latex');
% grid on;



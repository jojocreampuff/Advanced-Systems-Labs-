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
ref = @(t)[9.81*cos(0.2*t);       
             9.81*sin(0.2*t);    
             -4*cos(1*t)-9.81];

%% simualtion parameters (For testing controller)
global_parameters.W_nominal = 5; % wind speed for wind simulations (m/s)
sim_params.state_noise = 0.20;             % percent noise as a decimal
sim_params.control_noise = 1;           % percent noise as a decimal
sim_params.saturation_logic = 1;           % logical value to select saturation mode 0 = no, 1 = yes
sim_params.wind_logic = 1;                 % logical value to select wind noise mode (the same wind dist will be used in every case) 0 = no, 1 = yes 
sim_params.reference_smoothing_logic = 0;



%% Initial Condition
% offsets(:,1) = [ 0;  0; 0];
% offsets(:,2) = [-1;  0; 0];
% offsets(:,3) = [-1; -1; 0];
% offsets(:,4) = [-1;  1; 0];
IC1 = zeros(12,1);
IC2 = [1;-1;0; zeros(9,1)];
IC3 = [1;1;0; zeros(9,1)];
IC4 = [-1;1;0; zeros(9,1)];
IC_all = [IC1,IC2,IC3,IC4];

comm = 0;

tic
seed = randi(2^32 - 1);

Wxyz = gen_wind_vector(global_parameters.W_nominal,global_parameters.dt,global_parameters.tf, seed);

Flex_SNAC_results = simulate_Flex_SNAC_4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);
Offline_SNAC_results = simulate_Offline_SNAC_4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);
AC_results = simulate_AC_4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);
lqr_results = simulate_LQR_4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);
MPC_results = simulate_MPC_4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);
%% need matlab 2025 to run
% PPO_results = simulate_PPO4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);
% DDPG_results = simulate_DDPG_4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);

SMC_results = simulate_SMC_4drone(seed,comm,global_position, global_attitude, global_parameters,ref, IC_all, sim_params,Wxyz);

global_results.Flex_SNAC = Flex_SNAC_results;
global_results.SNAC = Offline_SNAC_results;
global_results.AC = AC_results;
global_results.MPC = MPC_results;
global_results.LQR = lqr_results;
global_results.SMC = SMC_results;
% global_results.PPO = PPO_results;
% global_results.DDPG = DDPG_results;
%% create a table and save as latex file
disp("#############+++++++++++ Simulation Parameters++++++++++++++++++++################")
fprintf("dt = %f, noise = [%f,%f], wind?? == %f, Nominal Wind == %f\n",global_parameters.dt,sim_params.state_noise,sim_params.control_noise,sim_params.wind_logic, global_parameters.W_nominal)
[metrics, names, Tmetrics] = extract_multiagent_control_metrics(global_results, global_parameters.dt);
disp(Tmetrics)
% save("ideal_case_again_again.mat")
% table_to_latex(Tmetrics, 'z_table5.tex');

%% Plot tracking error

%% flex SNAC
% figure;
% hold on;
% plot(Flex_SNAC_results.time, Flex_SNAC_results.tracking_error(1,:), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:500:length(T), 'MarkerSize', 5);
% plot(Flex_SNAC_results.time, Flex_SNAC_results.tracking_error(2,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:700:length(T), 'MarkerSize', 5);
% plot(Flex_SNAC_results.time, Flex_SNAC_results.tracking_error(3,:), 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:900:length(T), 'MarkerSize', 5);
% plot(Flex_SNAC_results.time, Flex_SNAC_results.tracking_error(4,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:1100:length(T), 'MarkerSize', 5);
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% % title('Flex SNAC Multi-Agent', 'Interpreter', 'latex');
% legend(["leader", "Drone 2", "Drone 3", "Drone 4"], 'Interpreter', 'latex');
% grid on;
% % SNAC
% figure;
% hold on;
% plot(Offline_SNAC_results.time, Offline_SNAC_results.tracking_error(1,:), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:500:length(T), 'MarkerSize', 5);
% plot(Offline_SNAC_results.time, Offline_SNAC_results.tracking_error(2,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:700:length(T), 'MarkerSize', 5);
% plot(Offline_SNAC_results.time, Offline_SNAC_results.tracking_error(3,:), 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:900:length(T), 'MarkerSize', 5);
% plot(Offline_SNAC_results.time, Offline_SNAC_results.tracking_error(4,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:1100:length(T), 'MarkerSize', 5);
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% % title('Offline SNAC Multi-Agent', 'Interpreter', 'latex');
% legend(["leader", "Drone 2", "Drone 3", "Drone 4"], 'Interpreter', 'latex');
% grid on;
% 
% figure;
% hold on;
% plot(Offline_SNAC_results.time, Offline_SNAC_results.formation_error_norm(:,2), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:500:length(T), 'MarkerSize', 5);
% plot(Flex_SNAC_results.time, Flex_SNAC_results.formation_error_norm(:,2), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:900:length(T), 'MarkerSize', 5);
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Collective Formation Error (m)', 'Interpreter', 'latex');
% % title('Offline SNAC Multi-Agent', 'Interpreter', 'latex');
% legend(["Non-flex", "Flex"], 'Interpreter', 'latex');
% grid on;

% % MPC
% figure;
% hold on;
% plot(MPC_results.time, MPC_results.tracking_error(1,:), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:100:length(T), 'MarkerSize', 5);
% plot(MPC_results.time, MPC_results.tracking_error(2,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:200:length(T), 'MarkerSize', 5);
% plot(MPC_results.time, MPC_results.tracking_error(3,:), 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:300:length(T), 'MarkerSize', 5);
% plot(MPC_results.time, MPC_results.tracking_error(4,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:400:length(T), 'MarkerSize', 5);
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% title('MPC Multi-Agent', 'Interpreter', 'latex');
% legend(["leader", "Drone 2", "Drone 3", "Drone 4"], 'Interpreter', 'latex');
% grid on;
% % SMC
% figure;
% hold on;
% plot(SMC_results.time, SMC_results.tracking_error(1,:), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:100:length(T), 'MarkerSize', 5);
% plot(SMC_results.time, SMC_results.tracking_error(2,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:200:length(T), 'MarkerSize', 5);
% plot(SMC_results.time, SMC_results.tracking_error(3,:), 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:300:length(T), 'MarkerSize', 5);
% plot(SMC_results.time, SMC_results.tracking_error(4,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:400:length(T), 'MarkerSize', 5);
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% title('SMC Multi-Agent', 'Interpreter', 'latex');
% legend(["leader", "Drone 2", "Drone 3", "Drone 4"], 'Interpreter', 'latex');
% grid on;
% % LQR
% figure;
% hold on;
% plot(lqr_results.time, lqr_results.tracking_error(1,:), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:100:length(T), 'MarkerSize', 5);
% plot(lqr_results.time, lqr_results.tracking_error(2,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:200:length(T), 'MarkerSize', 5);
% plot(lqr_results.time, lqr_results.tracking_error(3,:), 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:300:length(T), 'MarkerSize', 5);
% plot(lqr_results.time, lqr_results.tracking_error(4,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:400:length(T), 'MarkerSize', 5);
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% title('LQR Multi-Agent', 'Interpreter', 'latex');
% legend(["leader", "Drone 2", "Drone 3", "Drone 4"], 'Interpreter', 'latex');
% grid on;
% 
% %% AC
% figure;
% hold on;
% plot(AC_results.time, AC_results.tracking_error(1,:), 'LineWidth', 0.9, 'Marker', 'o', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:100:length(T), 'MarkerSize', 5);
% plot(AC_results.time, AC_results.tracking_error(2,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:200:length(T), 'MarkerSize', 5);
% plot(AC_results.time, AC_results.tracking_error(3,:), 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:300:length(T), 'MarkerSize', 5);
% plot(AC_results.time, AC_results.tracking_error(4,:), 'LineWidth', 0.9, 'Marker', '*', 'LineWidth', 0.9, ...
%     'MarkerIndices', 1:400:length(T), 'MarkerSize', 5);
% xlabel('Time (s)', 'Interpreter', 'latex');
% ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% title('AC Multi-Agent', 'Interpreter', 'latex');
% legend(["leader", "Drone 2", "Drone 3", "Drone 4"], 'Interpreter', 'latex');
% grid on;
%% Plot 3D trajectories of the drones
% figure;
% hold on; grid on; box on;
% 
% % Reference trajectory (leader reference)
% r_initial = zeros(3, length(T));
% for k = 1:length(T)
%     r_initial(:,k) = ref(T(k));
% end
% plot3(r_initial(1,:), r_initial(2,:), -r_initial(3,:), 'g--', 'LineWidth', 2);
% 
% % Drone trajectories
% plot3(Offline_SNAC_results.X(1,:,1), Offline_SNAC_results.X(2,:,1), -Offline_SNAC_results.X(3,:,1), ...
%     'k', 'LineWidth', 1.5);
% plot3(Offline_SNAC_results.X(1,:,2), Offline_SNAC_results.X(2,:,2), -Offline_SNAC_results.X(3,:,2), ...
%     'b', 'LineWidth', 1.5);
% plot3(Offline_SNAC_results.X(1,:,3), Offline_SNAC_results.X(2,:,3), -Offline_SNAC_results.X(3,:,3), ...
%     'r', 'LineWidth', 1.5);
% plot3(Offline_SNAC_results.X(1,:,4), Offline_SNAC_results.X(2,:,4), -Offline_SNAC_results.X(3,:,4), ...
%     'm', 'LineWidth', 1.5);
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('z (m)');
% title('SNAC: Multi-agent');
% legend('Reference', 'Leader', 'Drone 2', 'Drone 3', 'Drone 4', ...
%        'Start Leader', 'Start D2', 'Start D3', 'Start D4', ...
%        'Location', 'best');
% view(3);
% 

% r_initial = zeros(3, length(T));
% for k = 1:length(T)
%     r_initial(:,k) = ref(T(k));
% end
% 
% figure;
% hold on; grid on; box on;
% plot3(r_initial(1,:), r_initial(2,:), -r_initial(3,:), 'g--', 'LineWidth', 2);
% % Drone trajectories
% plot3(Flex_SNAC_results.X(1,:,1), Flex_SNAC_results.X(2,:,1), -Flex_SNAC_results.X(3,:,1), ...
%     'k', 'LineWidth', 1.5);
% plot3(Flex_SNAC_results.X(1,:,2), Flex_SNAC_results.X(2,:,2), -Flex_SNAC_results.X(3,:,2), ...
%     'b', 'LineWidth', 1.5);
% plot3(Flex_SNAC_results.X(1,:,3), Flex_SNAC_results.X(2,:,3), -Flex_SNAC_results.X(3,:,3), ...
%     'r', 'LineWidth', 1.5);
% plot3(Flex_SNAC_results.X(1,:,4), Flex_SNAC_results.X(2,:,4), -Flex_SNAC_results.X(3,:,4), ...
%     'm', 'LineWidth', 1.5);
% xlabel('x (m)');
% ylabel('y (m)');
% zlabel('z (m)');
% title('Flexible SNAC: Multi-agent');
% legend('Reference', 'Leader', 'Drone 2', 'Drone 3', 'Drone 4', ...
%        'Start Leader', 'Start D2', 'Start D3', 'Start D4', ...
%        'Location', 'best');
% view(3);
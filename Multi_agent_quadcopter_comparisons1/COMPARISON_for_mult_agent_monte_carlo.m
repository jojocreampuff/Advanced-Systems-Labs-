%% test mutli-agent in monte carlo for wind and noise
clear; clc; close all;
warning('off','all')
addpath('functions')
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
global_parameters.W_nominal = 15; % wind speed for wind simulations (m/s)
sim_params.state_noise = 0.2;             % percent noise as a decimal
sim_params.control_noise = 1;           % percent noise as a decimal
sim_params.saturation_logic = 1;           % logical value to select saturation mode 0 = no, 1 = yes
sim_params.wind_logic = 1;                 % logical value to select wind noise mode (the same wind dist will be used in every case) 0 = no, 1 = yes 
sim_params.reference_smoothing_logic = 0;

%% Initial Condition
% IC1 = [0;0;0;zeros(9,1)];
% IC2 = [-1;0;0; zeros(9,1)];
% IC3 = [-1;-1;0; zeros(9,1)]; 
% IC4 = [-1;1;0; zeros(9,1)];
IC1 = zeros(12,1);
IC2 = [1;-1;0; zeros(9,1)];
IC3 = [1;1;0; zeros(9,1)];
IC4 = [-1;1;0; zeros(9,1)];
IC_all_nom = [IC1,IC2,IC3,IC4];

tic
comm = 1;
for l = 1:5
    fprintf("Run Number %f\n", l)
    seed = randi(2^32 - 1)
    IC = [(5 - - 5).*rand(3,4) + -5; (5 - - 5).*rand(3,4) + -5; (pi/5 - - pi/5).*rand(3,4) + -pi/5; (pi/5 - - pi/5).*rand(3,4) + -pi/5];
    IC_all = IC_all_nom + IC;
    Wxyz = gen_wind_vector(global_parameters.W_nominal,global_parameters.dt,global_parameters.tf, seed);
    % mc_results.PPO.run(l) = simulate_PPO4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);
    mc_results.Flex_SNAC.run(l) = simulate_Flex_SNAC_4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);
    mc_results.SNAC.run(l) = simulate_Offline_SNAC_4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);
    mc_results.AC.run(l) = simulate_AC_4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);
    mc_results.LQR.run(l) = simulate_LQR_4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);
    mc_results.MPC.run(l) = simulate_MPC_4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);
    % mc_results.DDPG.run(l) = simulate_DDPG_4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);
    mc_results.SMC.run(l) = simulate_SMC_4drone(seed,comm,global_position, global_attitude, global_parameters, ref, IC_all, sim_params, Wxyz);

end
total_time = toc
[metrics_table] = extract_mc_multiagent_metrics(mc_results, global_parameters.dt);
disp(metrics_table)
table_to_latex(metrics_table,"z_DDGP_multi_agent_MC_25_runs_random_IC_batch1.tex")

save("z_DDPG_multi_agent_MC_25_runs_random_IC_batch1.mat")
%% test all controllers in monte carlo for wind and noise
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
sim_params.state_noise = 0.20;             % percent noise as a decimal
sim_params.control_noise = 1;           % percent noise as a decimal
sim_params.saturation_logic = 1;           % logical value to select saturation mode 0 = no, 1 = yes
sim_params.wind_logic = 0;                 % logical value to select wind noise mode (the same wind dist will be used in every case) 0 = no, 1 = yes 
sim_params.reference_smoothing_logic = 0;


for l = 1:30
    fprintf("Run Number %f\n", l)
    seed = randi(2^32 - 1);
    IC = [(5 - - 5).*rand(3,1) + -5; (5 - - 5).*rand(3,1) + -5; (pi/5 - - pi/5).*rand(3,1) + -pi/5; (pi/5 - - pi/5).*rand(3,1) + -pi/5];
    Wxyz = gen_wind_vector(global_parameters.W_nominal,global_parameters.dt,global_parameters.tf, seed);
    seed_vec(l) = seed;
    ICs(:,l) = IC;

    mc_results.Flex_SNAC.run(l) = simulate_Flex_SNAC(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.SNAC.run(l) = simulate_Offline_SNAC(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.AC.run(l) = simulate_Offline_AC(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.LQR.run(l) = simulate_LQR(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.MPC.run(l) = simulate_MPC(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    % mc_results.DDPG.run(l) = simulate_DDPG(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.PID.run(l) = simulate_PID(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.SMC.run(l) = simulate_SMC(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.FL.run(l) = simulate_FL(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);
    mc_results.Backstepping.run(l) = simulate_Backstepping(seed,global_position, global_attitude, global_parameters, ref, IC, sim_params, Wxyz);

end
% save("z_30_MC_runs_single_drone.mat","mc_results")
[mc_stats, controller_names, metrics_table] = extract_monte_carlo_metrics(mc_results, global_parameters.dt);
disp(mc_stats)
disp(metrics_table)
% table_to_latex(metrics_table,"Table_2_case_1_final.tex")

% save("monte_carlo_run_withPPO_best.mat")
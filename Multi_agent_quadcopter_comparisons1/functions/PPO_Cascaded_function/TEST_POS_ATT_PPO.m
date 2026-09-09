clear; clc; close all;
%PARAMETER
Ts_Sim = 0.01 ;  % best time step 0.01
Tf = 50;
m = 1; 
g = 9.81;
mdl = "ppo_position_attitude.mdl";

load("test_2_trainedPPOAgent_position.mat", "agent");
agent.UseExplorationPolicy = false;
agent.SampleTime = Ts_Sim;
load("pretrainedPPOAgent_attitude.mat", "agent1");
agent1.UseExplorationPolicy = false;
agent1.SampleTime = Ts_Sim;

state_noise = 0.045;
control_noise = 0.045;

W_nominal = 11; %m/s
Control_Saturation = 1;  % 1 mean on saturation

T = (0:Ts_Sim:Tf-Ts_Sim)';     % Nx1 column time (or start at 0, see note below)
N = length(T);
Wxyz = gen_wind_vector(W_nominal, Ts_Sim, Tf);  % should be 3xN

u = Wxyz.';
Wxyz_timeseries = [T u];

%initial conditions
IC = [1.1; -1.1; 0; 0; 0; 0; 0; -0; 0; -0; -0.; 0.]; % hard
x_IC = IC(1);
y_IC = IC(2);
z_IC = IC(3);

u_IC = IC(4);
v_IC = IC(5);
w_IC =IC(6);

phi_IC = IC(7);
theta_IC = IC(8);
psi_IC = IC(9);

p_IC = IC(10);
q_IC = IC(11);
r_IC = IC(12);

tic
simOut = sim('ppo_position_attitude');
PPO_sim_time = toc;

disp(['Simulation runtime: ', num2str(PPO_sim_time), ' seconds']) % 17.9222 seconds

X = squeeze(simOut.X);
U = squeeze(simOut.U);
U_pos = squeeze(simOut.U_pos);
X_ref = simOut.X_ref';
pos_error = simOut.pos_error';
att_error = squeeze(simOut.att_error);

Q_pos = diag([1,1,1,1,1,1])*1; % continous time parameters (PLEASE CHECK IF YOUR WORKING IN DISCRETE OR CONT)
R_pos = diag([1,1,1])*1;
Q_att = diag([1,1,1,1,1,1])*1;
R_att = diag([1,1,1])*1;

pos_tracking_cost = Ts_Sim * sum( sum( pos_error .* (Q_pos*pos_error), 1 ) );
pos_control_cost = Ts_Sim  * sum( sum( U_pos .* (R_pos*U_pos), 1 ) );

att_tracking_cost = Ts_Sim * sum( sum( att_error .* (Q_att*att_error), 1 ) );
att_control_cost = Ts_Sim  * sum( sum( U(2:4,:) .* (R_att*U(2:4,:)), 1 ) );

PPO_results.PPO_c_cost_total = pos_tracking_cost + pos_control_cost + att_control_cost + att_tracking_cost;

% save all useful info in a struck
PPO_results.X = X;
PPO_results.U = U;
PPO_results.X_ref = X_ref;
PPO_results.error = [pos_error;att_error];
PPO_results.simtime = PPO_sim_time;
PPO_error = PPO_results.error;
PPO_tracking_error = (PPO_error(1,:).^2 + PPO_error(2,:).^2 + PPO_error(3,:).^2).^0.5;
PPO_results.PPO_tracking_error = PPO_tracking_error;
PPO_results.PPO_tracking_error_norm = Ts_Sim*sum(PPO_tracking_error,2);

save('PPO_simulation.mat','PPO_results', 'Ts_Sim', 'state_noise', 'control_noise', 'W_nominal', 'Control_Saturation');

% disp("======= PPO Results ======")
fprintf("PPO simulation time: %.4f\n", PPO_results.simtime);
fprintf("PPO total tracking error: %.4f\n", PPO_results.PPO_tracking_error_norm);
fprintf("PPO tracking cost: %.4f\n", PPO_results.PPO_c_cost_total(end));

figure;
hold on;
plot(T(1:end-1), PPO_results.PPO_tracking_error(1,1:N-1), 'k', 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5);
ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% ylim([0 1])
title('L2 Norm Tracking Error', 'Interpreter', 'latex');
% legend(["PPO","SMC","FL","Offline AC","Offline SNAC","Online AC","Online SNAC","Online Single Critic"], 'Interpreter', 'latex');
grid on;

figure;
grid on
hold on
refxyz = PPO_results.X_ref(1:6,:);
plot3(refxyz(1,:), refxyz(2,:), -refxyz(3,:), '-', 'Color', [0 1 0], 'Linewidth', .9)
% PPO
plot3(PPO_results.X(1,:), PPO_results.X(2,:), -PPO_results.X(3,:), 'k', 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5)
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
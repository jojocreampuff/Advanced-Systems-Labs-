%% Comparison script for noise case : 
% load all .mat files from every simulation in here after checking that Q,
% R, dt, IC, and other parameters are the same 
% compare SNAC to, PID, LQR, MPC, AC
% plot: tracking across each controller
        % Cost across every controller
        % controls across every controller

% TO DO:

clear; clc; close all;
dt = 0.004;
%% load SNAC STUFF
% position training time 21.8675 seconds w/ 1000 samples and 5000 iter
% attitude training time 103.223 seconds w/ 1000 samples and 10000 iterations
% trajectory 1 is the one with the correct ICs
load("SNAC_compare_workspace_noise.mat")
snac_results = SNAC_results;
SNAC_IC1_traj = snac_results.X; % IC is 5, 5, 0 use this IC for all cost comparisons
SNAC_error = snac_results.error;
SNAC_in_cost = SNAC_results.inscost;
SNAC_c_cost = SNAC_results.cumcost;
SNAC_in_cost_total = sum(SNAC_in_cost,1);
SNAC_c_cost_total = sum(SNAC_c_cost, 1);
SNAC_control = snac_results.U;
refxyz = snac_results.r_initial;
SNAC_tracking_error = (SNAC_error(1,:).^2 + SNAC_error(2,:).^2 + SNAC_error(3,:).^2).^0.5;
SNAC_tracking_error_sum = dt*sum(SNAC_tracking_error,2);
SNAC_time = SNAC_results.simtime;

t_f = 50;
N = t_f/dt;
T = 0:dt:t_f-dt;

%% load PID STUFFAC
load("PID_compare_workspace_noise.mat")
pid_traj = PID_results.X; % IC is 5, 5, 0 use this IC for all cost comparisons
PID_error = -PID_results.error;
PID_tracking_error = (PID_error(1,:).^2 + PID_error(2,:).^2 + PID_error(3,:).^2).^0.5;
PID_tracking_error_sum = dt*sum(PID_tracking_error,2);
PID_in_cost = PID_results.inscost;
PID_c_cost = PID_results.cumcost;
PID_in_cost_total = sum(PID_in_cost,1);
PID_c_cost_total = sum(PID_c_cost, 1);
PID_control = PID_results.U;
PID_time = PID_results.simtime;

%% load LQR stuff
load("lqr_compare_workspace_noise.mat")
lqr_traj = lqr_results.X; % IC is 5, 5, 0 use this IC for all cost comparisons
lqr_error = lqr_results.error;
lqr_tracking_error = (lqr_error(1,:).^2 + lqr_error(2,:).^2 + lqr_error(3,:).^2).^0.5;
lqr_tracking_error_sum = dt*sum(lqr_tracking_error,2);
lqr_in_cost = lqr_results.inscost;
lqr_c_cost = lqr_results.cumcost;
lqr_in_cost_total = sum(lqr_in_cost,1);
lqr_c_cost_total = sum(lqr_c_cost, 1);
lqr_control = lqr_results.U;
lqr_time = lqr_results.simtime;

%% load AC stuff
% position training time 42.065000 seconds w/ 1000 samples and 5000 iterations
% attitude training time 503.910601 seconds w/ 1000 samples and 10000 iterations
load("AC_compare_workspace_noise.mat")
AC_traj = AC_results.X; % IC is 5, 5, 0 use this IC for all cost comparisons
AC_error = AC_results.error;
AC_tracking_error = (AC_error(1,:).^2 + AC_error(2,:).^2 + AC_error(3,:).^2).^0.5;
AC_tracking_error_sum = dt*sum(AC_tracking_error,2);
AC_in_cost = AC_results.inscost;
AC_c_cost = AC_results.cumcost;
AC_in_cost_total = sum(AC_in_cost,1);
AC_c_cost_total = sum(AC_c_cost, 1);
AC_control = AC_results.U;
AC_time = AC_results.simtime;

%% load MPC stuff
load("MPC_compare_workspace_noise.mat")
MPC_traj = MPC_results.X; % IC is 5, 5, 0 use this IC for all cost comparisons
MPC_error = MPC_results.error;
MPC_tracking_error = (MPC_error(1,:).^2 + MPC_error(2,:).^2 + MPC_error(3,:).^2).^0.5;
MPC_tracking_error_sum = dt*sum(MPC_tracking_error,2);
MPC_in_cost = MPC_results.inscost;
MPC_c_cost = MPC_results.cumcost;
MPC_in_cost_total = sum(MPC_in_cost,1);
MPC_c_cost_total = sum(MPC_c_cost, 1);
MPC_control = MPC_results.U;
MPC_time = MPC_results.simtime;

%% Get stat data
SNAC_MEAN_ERROR = mean(SNAC_tracking_error);
SNAC_variance_ERROR = var(SNAC_tracking_error);
SNAC_STD_ERROR = std(SNAC_tracking_error);
SNAC_MEAN_STD_VAR_ERROR = [SNAC_MEAN_ERROR,SNAC_STD_ERROR,SNAC_variance_ERROR];

SNAC_MEAN_cost = mean(SNAC_c_cost_total(3:end));
SNAC_variance_cost = var(SNAC_c_cost_total(3:end));
SNAC_STD_cost = std(SNAC_c_cost_total(3:end));
SNAC_MEAN_STD_VAR_cost = [SNAC_MEAN_cost,SNAC_STD_cost,SNAC_variance_cost];

SNAC_MEAN_incost = mean(SNAC_in_cost_total(2:end));
SNAC_variance_incost = var(SNAC_in_cost_total(2:end));
SNAC_STD_incost = std(SNAC_in_cost_total(2:end));
SNAC_MEAN_STD_VAR_incost = [SNAC_MEAN_incost,SNAC_STD_incost,SNAC_variance_incost];

z_SNAC_MEAN_STD_VAR_error_cost_incost = [SNAC_MEAN_STD_VAR_ERROR;SNAC_MEAN_STD_VAR_cost; SNAC_MEAN_STD_VAR_incost];

AC_MEAN_ERROR = mean(AC_tracking_error);
AC_variance_ERROR = var(AC_tracking_error);
AC_STD_ERROR = std(AC_tracking_error);
AC_MEAN_STD_VAR_ERROR = [AC_MEAN_ERROR,AC_STD_ERROR,AC_variance_ERROR];

AC_MEAN_cost = mean(AC_c_cost_total(3:end));
AC_variance_cost = var(AC_c_cost_total(3:end));
AC_STD_cost = std(AC_c_cost_total(3:end));
AC_MEAN_STD_VAR_cost = [AC_MEAN_cost,AC_STD_cost,AC_variance_cost];

AC_MEAN_incost = mean(AC_in_cost_total(2:end));
AC_variance_incost = var(AC_in_cost_total(2:end));
AC_STD_incost = std(AC_in_cost_total(2:end));
AC_MEAN_STD_VAR_incost = [AC_MEAN_incost,AC_STD_incost,AC_variance_incost];
z_AC_MEAN_STD_VAR_error_cost_incost = [AC_MEAN_STD_VAR_ERROR;AC_MEAN_STD_VAR_cost; AC_MEAN_STD_VAR_incost];

lqr_MEAN_ERROR = mean(lqr_tracking_error);
lqr_variance_ERROR = var(lqr_tracking_error);
lqr_STD_ERROR = std(lqr_tracking_error);
lqr_MEAN_STD_VAR_ERROR = [lqr_MEAN_ERROR,lqr_STD_ERROR,lqr_variance_ERROR];

lqr_MEAN_cost = mean(lqr_c_cost_total(3:end));
lqr_variance_cost = var(lqr_c_cost_total(3:end));
lqr_STD_cost = std(lqr_c_cost_total(3:end));
lqr_MEAN_STD_VAR_cost = [lqr_MEAN_cost,lqr_STD_cost,lqr_variance_cost];

lqr_MEAN_incost = mean(lqr_in_cost_total(2:end));
lqr_variance_incost = var(lqr_in_cost_total(2:end));
lqr_STD_incost = std(lqr_in_cost_total(2:end));
lqr_MEAN_STD_VAR_incost = [lqr_MEAN_incost,lqr_STD_incost,lqr_variance_incost];

z_lqr_MEAN_STD_VAR_error_cost_incost = [lqr_MEAN_STD_VAR_ERROR;lqr_MEAN_STD_VAR_cost; lqr_MEAN_STD_VAR_incost];

MPC_MEAN_ERROR = mean(MPC_tracking_error);
MPC_variance_ERROR = var(MPC_tracking_error);
MPC_STD_ERROR = std(MPC_tracking_error);
MPC_MEAN_STD_VAR_ERROR = [MPC_MEAN_ERROR,MPC_STD_ERROR,MPC_variance_ERROR];

MPC_MEAN_cost = mean(MPC_c_cost_total(3:end));
MPC_variance_cost = var(MPC_c_cost_total(3:end));
MPC_STD_cost = std(MPC_c_cost_total(3:end));
MPC_MEAN_STD_VAR_cost = [MPC_MEAN_cost,MPC_STD_cost,MPC_variance_cost];

MPC_MEAN_incost = mean(MPC_in_cost_total(2:end));
MPC_variance_incost = var(MPC_in_cost_total(2:end));
MPC_STD_incost = std(MPC_in_cost_total(2:end));
MPC_MEAN_STD_VAR_incost = [MPC_MEAN_incost,MPC_STD_incost,MPC_variance_incost];
z_MPC_MEAN_STD_VAR_error_cost_incost = [MPC_MEAN_STD_VAR_ERROR; MPC_MEAN_STD_VAR_cost; MPC_MEAN_STD_VAR_incost];

PID_MEAN_ERROR = mean(PID_tracking_error);
PID_variance_ERROR = var(PID_tracking_error);
PID_STD_ERROR = std(PID_tracking_error);
PID_MEAN_STD_VAR_ERROR = [PID_MEAN_ERROR,PID_STD_ERROR,PID_variance_ERROR];

PID_MEAN_cost = mean(PID_c_cost_total(3:end));
PID_variance_cost = var(PID_c_cost_total(3:end));
PID_STD_cost = std(PID_c_cost_total(3:end));
PID_MEAN_STD_VAR_cost = [PID_MEAN_cost,PID_STD_cost,PID_variance_cost];

PID_MEAN_incost = mean(PID_in_cost_total(2:end));
PID_variance_incost = var(PID_in_cost_total(2:end));
PID_STD_incost = std(PID_in_cost_total(2:end));
PID_MEAN_STD_VAR_incost = [PID_MEAN_incost,PID_STD_incost,PID_variance_incost];

z_PID_MEAN_STD_VAR_error_cost_incost = [PID_MEAN_STD_VAR_ERROR;PID_MEAN_STD_VAR_cost; PID_MEAN_STD_VAR_incost];

%% display stuff
disp("Results of 50 Seconds of simulation on the same trajectory")
disp("======= SNAC Results ======")
disp("Position controller training time: 21.8675 seconds w/ 1000 samples and 5000 iterations")
disp("Attitude controller training time: 55.1765 seconds w/ 1000 samples and 10000 iterations")
fprintf("SNAC simulation time: %.4f\n", SNAC_time);
fprintf("SNAC total tracking error: %.4f\n", SNAC_tracking_error_sum);
fprintf("SNAC tracking cost: %.4f\n", SNAC_c_cost_total(end) - SNAC_c_cost_total(2))
fprintf("SNAC max pitch angle: %.4f\n", rad2deg(max(abs(SNAC_IC1_traj(7,:)))))
fprintf("SNAC max pitch roll: %.4f\n", rad2deg(max(abs(SNAC_IC1_traj(8,:)))))
fprintf("SNAC max pitch yaw: %.4f\n", rad2deg(max(abs(SNAC_IC1_traj(9,:)))))
disp("======= AC Results ======")
disp("Position controller training time: 42.065 seconds w/ 1000 samples and 5000 iterations")
disp("Attitude controller training time: 551.948500 seconds w/ 1000 samples and 10000 iterations")
fprintf("AC simulation time: %.4f\n", AC_time);
fprintf("AC total tracking error: %.4f\n", AC_tracking_error_sum);
fprintf("AC tracking cost: %.4f\n", AC_c_cost_total(end)- AC_c_cost_total(2));
fprintf("AC max pitch angle: %.4f\n", rad2deg(max(abs(AC_traj(7,:)))))
fprintf("AC max pitch roll: %.4f\n", rad2deg(max(abs(AC_traj(8,:)))))
fprintf("AC max pitch yaw: %.4f\n", rad2deg(max(abs(AC_traj(9,:)))))
disp("======= LQR Results ======")
fprintf("LQR simulation time: %.4f\n", lqr_time);
fprintf("LQR total tracking error: %.4f\n", lqr_tracking_error_sum);
fprintf("LQR tracking cost: %.4f\n", lqr_c_cost_total(end)- lqr_c_cost_total(2));
fprintf("LQR max pitch angle: %.4f\n", rad2deg(max(abs(lqr_traj(7,:)))))
fprintf("LQR max pitch roll: %.4f\n", rad2deg(max(abs(lqr_traj(8,:)))))
fprintf("LQR max pitch yaw: %.4f\n", rad2deg(max(abs(lqr_traj(9,:)))))
disp("======= MPC Results ======")
fprintf("MPC simulation time: %.4f\n", MPC_time);
fprintf("MPC total tracking error: %.4f\n", MPC_tracking_error_sum);
fprintf("MPC tracking cost: %.4f\n", MPC_c_cost_total(end)- MPC_c_cost_total(2));
fprintf("MPC max pitch angle: %.4f\n", rad2deg(max(abs(MPC_traj(7,:)))))
fprintf("MPC max pitch roll: %.4f\n", rad2deg(max(abs(MPC_traj(8,:)))))
fprintf("MPC max pitch yaw: %.4f\n", rad2deg(max(abs(MPC_traj(9,:)))))
disp("======= PID Results ======")
fprintf("PID simulation time: %.4f\n", PID_time);
fprintf("PID total tracking error: %.4f\n", PID_tracking_error_sum);
fprintf("PID tracking cost: %.4f\n", PID_c_cost_total(end) - PID_c_cost_total(2));
fprintf("PID max pitch angle: %.4f\n", rad2deg(max(abs(pid_traj(7,:)))))
fprintf("PID max pitch roll: %.4f\n", rad2deg(max(abs(pid_traj(8,:)))))
fprintf("PID max pitch yaw: %.4f\n", rad2deg(max(abs(pid_traj(9,:)))))

%% plot stuff
figure;
grid on
hold on

% Reference
plot3(refxyz(1,:), refxyz(2,:), -refxyz(3,:), '--', 'Color', [0 1 0], 'Linewidth', .9)
% SNAC
plot3(SNAC_IC1_traj(1,:), SNAC_IC1_traj(2,:), -SNAC_IC1_traj(3,:), 'r', 'Marker', 'x', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(pid_traj(1,:)), 'MarkerSize', 5)
% AC
plot3(AC_traj(1,:), AC_traj(2,:), -AC_traj(3,:), '--', ...
    'Color', [0 0 1], 'Marker', 's', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1100:length(AC_traj(1,:)), 'MarkerSize', 5)
% LQR
plot3(lqr_traj(1,:), lqr_traj(2,:), -lqr_traj(3,:), 'k', 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1200:length(AC_traj(1,:)), 'MarkerSize', 5)
% MPC
plot3(MPC_traj(1,1:10:end), MPC_traj(2,1:10:end), -MPC_traj(3,1:10:end), '-.', ...
    'Color', [1 0 1], 'Marker', 'o', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:200:length(MPC_traj(1,:)), 'MarkerSize', 5)
% PID
plot3(pid_traj(1,:), pid_traj(2,:), -pid_traj(3,:), '-', ...
    'Color', [0 0.6 0], 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1500:length(pid_traj(1,:)), 'MarkerSize', 5)

xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
legend(["$r(t)$", "SNAC", "AC", "LQR", "MPC", "PID"], 'Interpreter', 'latex');

%% Tracking Error
figure;
hold on;
% SNAC
plot(T(1:end-1), SNAC_tracking_error(1,1:N-1), 'r', 'Marker', 'x', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(pid_traj(1,:)), 'MarkerSize', 5);
% AC
plot(T(1:end-1), AC_tracking_error(1,1:N-1), '--', ...
    'Color', [0 0 1], 'Marker', 's', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1100:length(AC_traj(1,:)), 'MarkerSize', 5);
% LQR
plot(T(1:end-1), lqr_tracking_error(1,1:N-1), 'k', 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1200:length(AC_traj(1,:)), 'MarkerSize', 5);
% MPC
plot(T(1:10:end-1), MPC_tracking_error(1,1:10:N-1), '-.', ...
    'Color', [1 0 1], 'Marker', 'o', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:200:length(MPC_traj(1,:)), 'MarkerSize', 5);
% PID
plot(T(1:end-1), PID_tracking_error(1,1:N-1), '-', ...
    'Color', [0 0.6 0], 'Marker', '*', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1500:length(pid_traj(1,:)), 'MarkerSize', 5);

set(gca,'linewidth',1);
set(gca,'GridLineStyle',':')
set(gca,'GridAlpha',0.7)
set(gca,'GridColor',[0 0 0])
xlabel('Time (s)', 'Interpreter', 'latex');
ylabel('Tracking Error [m]', 'Interpreter', 'latex');
ylim([0 5])
legend("SNAC", "AC", "LQR", "MPC", "PID", 'Interpreter', 'latex');
grid on;


%% plot cost
figure;
hold on;
plot(T(1:end-1), SNAC_in_cost_total(1,1:N-1), 'LineWidth', 1.5);
plot(T(1:end-1), AC_in_cost_total(1,1:N-1), 'LineWidth', 1.5);
plot(T(1:end-1),lqr_in_cost_total(1,1:N-1), 'LineWidth', 1.5);
plot(T(1:end-1),MPC_in_cost_total(1,1:N-1), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Instantaneous Cost');
% ylim([0 4e5])
title('Total Instantaneous Cost Over Time');
legend("SNAC Control","AC Control","LQR Control","MPC Control")
grid on;

figure;
hold on;
plot(T(1:end-1), SNAC_c_cost_total(1,1:N-1), 'LineWidth', 1.5);
plot(T(1:end-1), AC_c_cost_total(1,1:N-1), 'LineWidth', 1.5);
plot(T(1:end-1),lqr_c_cost_total(1,1:N-1), 'LineWidth', 1.5);
plot(T(1:end-1),MPC_c_cost_total(1,1:N-1), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Cumulative Cost');
title('Cumulative Cost Over Time');
legend("SNAC Control","AC Control","LQR Control","MPC Control")
grid on;


clear; clc; close all;
%% this will pllt outside safe barrier function for the paper
load("RLS_SNAC_MF_GP.mat")
params.T_sim = 11;
params.dt    = 0.01;
% saveFigures("RLS_SNAC_MF_GP_NO_Safety")
RLS_SNAC_tracking_sim  = simulate_closed_loop(RLS_SNAC_MF.W_final, params);

fprintf("\n=== Closed-loop Tracking RMSE ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.RMSE);

fprintf("\n=== Closed-loop Tracking Control Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.control_cost);

fprintf("\n=== Closed-loop Tracking Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.tracking_cost);

case3_color = [1 0 1];
case1_marker = 50;
case2_marker = 70;
mark_size = 3;

plot_training(RLS_SNAC_MF.t',RLS_SNAC_MF.err',RLS_SNAC_MF.theta_hist',"RLS Training",10)
plot_GP_drift_stuff(RLS_SNAC_MF,   "Drift Approx  and GP", 200);

%% =======================
%  Plots
% =======================
REE = 500;

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(RLS_SNAC_tracking_sim.x(1,1:end-REE), RLS_SNAC_tracking_sim.x(2,1:end-REE),'b-','Marker', 'o', 'LineWidth', 1, ...
    'MarkerIndices', 1:case2_marker:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', mark_size);
plot(RLS_SNAC_tracking_sim.ref(1,1:end-REE), RLS_SNAC_tracking_sim.ref(2,1:end-REE),'g--');
xlabel('$x_1$', 'Interpreter','latex'); ylabel('$x_2$', 'Interpreter','latex');
% xlim([-1.5 1.5])
% ylim([-2 2])
legend({'RLS SNAC','$r(t)$'},...
        'Interpreter','latex');

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(RLS_SNAC_tracking_sim.time(:,1:end-REE), RLS_SNAC_tracking_sim.u(:,1:end-REE),'b-','Marker', 'o', 'LineWidth', 1, ...
    'MarkerIndices', 1:case2_marker:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', mark_size);
xlabel("Time (s)",'Interpreter','latex'); ylabel("$u(t)$",'Interpreter','latex');


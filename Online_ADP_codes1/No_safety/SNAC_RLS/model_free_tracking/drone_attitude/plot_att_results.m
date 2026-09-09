clear; clc; close all;
%% plot stuff
load("SNAC_RLS_att_3.mat")
%%  Plot training (use the step size to reduce figure filesize)
plot_training(RLS_SNAC_MF.t',RLS_SNAC_MF.err,RLS_SNAC_MF.theta_hist',RLS_SNAC_MF.P_norm',"RLS Training",10)
plot_GP_drift_stuff(RLS_SNAC_MF,   "Drift Approx  and GP", 200);
%%  Run time: Closed-loop sim (ode45)

fprintf("\n=== Closed-loop Tracking RMSE ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.RMSE);

fprintf("\n=== Closed-loop Tracking Control Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.control_cost);

fprintf("\n=== Closed-loop Tracking Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.tracking_cost);

% =======================
%  Plots
% =======================
figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot3(RLS_SNAC_tracking_sim.x(1,:),   RLS_SNAC_tracking_sim.x(2,:), RLS_SNAC_tracking_sim.x(3,:), 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:500:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
plot3(RLS_SNAC_tracking_sim.ref(1,:), RLS_SNAC_tracking_sim.ref(2,:), RLS_SNAC_tracking_sim.ref(3,:),"Color","g","LineStyle","-","LineWidth",1);
xlabel("$\phi$", Interpreter="latex"); ylabel("$\theta$", Interpreter="latex"); zlabel("$\psi$", Interpreter="latex");
% xlim([-1.5 1.5])
% ylim([-2 2])
legend("RLS SNAC","Reference",'Location','best', Interpreter="latex");
title("State-space tracking");

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(RLS_SNAC_tracking_sim.time,   RLS_SNAC_tracking_sim.L2_err, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("error");
% ylim([0 .2])
title("Error plot");

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(RLS_SNAC_tracking_sim.time,   RLS_SNAC_tracking_sim.u, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("Control");
title("Control Plot");

% saveFigures("drone_att_training")
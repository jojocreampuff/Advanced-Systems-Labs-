clear; clc; close all;
% close all;
%% this will pllt outside safe barrier function for the paper
load("seconds_good_one_barrier2.mat")
% saveFigures("model_free_barrier2_new")

% for i = 1:length(trainSafe.x_hist(:,1))
%     [~, ~, h(i)] = B_x(trainSafe.x_hist(i,:), params) ;
% end
% 
% figure;
% hold on; grid on; box on;
% set(gca, 'GridLineStyle', ':');
% plot(trainSafe.t', h);
% yline(0,'r-');
% xlabel("Time (s)","Interpreter","latex"); ylabel("$h(x)$","Interpreter","latex");
% ylim([0 .1])

%% barrier information in these results
% h(x) = (x(1)-c(1)).^2 + (x(2)-c(2)).^2 - r^2 > 0
% B = gamma/h(x);
% ub = -cb * G(x)^T * gradB;

fprintf("\n=== Closed-loop Tracking RMSE ===\n");
fprintf("  Case 1: Unsafe policy: %.6f\n", Case1_sim.RMSE);
fprintf("  Case 2: Safe   policy w/ safe exploration: %.6f\n", Case2_sim.RMSE);
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %.6f\n", Case3_sim.RMSE);

fprintf("\n=== Closed-loop Tracking Control Cost ===\n");
fprintf("  Case 1: Unsafe policy: %.6f\n", Case1_sim.control_cost);
fprintf("  Case 2: Safe   policy w/ safe exploration: %.6f\n", Case2_sim.control_cost);
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %.6f\n", Case3_sim.control_cost);

fprintf("\n=== Closed-loop Tracking Cost ===\n");
fprintf("  Case 1: Unsafe policy: %.6f\n", Case1_sim.tracking_cost);
fprintf("  Case 2: Safe   policy w/ safe exploration: %.6f\n", Case2_sim.tracking_cost);
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %.6f\n", Case3_sim.tracking_cost);

fprintf("\n=== Safety Violations (h < 0 in X timesteps) ===\n");
fprintf("  Case 1: Unsafe policy: %d / %d\n", Case1_sim.safety_viol, length(Case1_sim.h_log));
fprintf("  Case 2: Safe   policy: %d / %d\n", Case2_sim.safety_viol,   length(Case2_sim.h_log));
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %d / %d\n", Case3_sim.safety_viol,   length(Case3_sim.h_log));

case3_color = [1 0 1];
case1_marker = 50;
case2_marker = 70;
mark_size = 3;
%% =======================
%  Plots
% =======================
%% =======================
%  Plot training (use the step size to reduce figure filesize)
% plot_training(trainUnsafe.t',trainUnsafe.err',trainUnsafe.theta_hist',"Training Without Safety",100)
% plot_training(trainSafe.t',trainSafe.err',trainSafe.theta_hist',"Training With Safety", 200)
%% =======================
%  Extra subplot diagnostics
% plot_GP_drift_stuff(trainSafe,   "Drift Approx During SAFE Exploration Training", 100);
% plot_GP_drift_stuff(trainUnsafe,   "Drift Approx During UNSAFE Exploration Training", 200);

REE = 500;
figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(Case1_sim.time(:,1:end-REE), Case1_sim.h_log(:,1:end-REE),'k--');
plot(Case2_sim.time(:,1:end-REE),   Case2_sim.h_log(:,1:end-REE),'b-');
plot(Case3_sim.time(:,1:end-REE), Case3_sim.h_log(:,1:end-REE),'Color', case3_color, 'LineStyle','-.'); 
yline(0,'r-');
xlabel("Time (s)", 'Interpreter','latex');
ylabel("$h(x)$",   'Interpreter','latex');

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(Case1_sim.x(1,1:end-REE), Case1_sim.x(2,1:end-REE),'k--');
plot(Case2_sim.x(1,1:end-REE), Case2_sim.x(2,1:end-REE),'b-');
plot(Case3_sim.x(1,1:end-REE), Case3_sim.x(2,1:end-REE),'Color', case3_color, 'LineStyle','-.');
plot(Case1_sim.ref(1,1:end-REE), Case1_sim.ref(2,1:end-REE),'g');
plot(bx, by,'r-');
fill(bx, by, 'r', ...
    'FaceAlpha', 0.10, ...
    'EdgeColor', 'none', ...
    'HandleVisibility', 'off');
xlabel('$x_1$', 'Interpreter','latex'); ylabel('$x_2$', 'Interpreter','latex');
xlim([-1.5 1.5])
ylim([-2 2])
legend({'Unsafe', ...
        'Safe', ...
        'Shielded', ...
        '$r(t)$', ...
        '$h(x)=0$'},...
        'Interpreter','latex');

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(Case1_sim.time(:,1:end-REE), Case1_sim.u(:,1:end-REE),'k--');
plot(Case2_sim.time(:,1:end-REE),   Case2_sim.u(:,1:end-REE),'b-');
plot(Case3_sim.time(:,1:end-REE), Case3_sim.u(:,1:end-REE),'Color', case3_color, 'LineStyle','-.');
xlabel("Time (s)",'Interpreter','latex'); ylabel("$u(t)$",'Interpreter','latex');


% 
% 
% %% =======================
% %  Plots
% % =======================
% REE = 500;
% % figure; 
% % subplot(1,3,1)
% % hold on; grid on; box on;
% % set(gca, 'GridLineStyle', ':');
% % plot(Case1_sim.time(:,1:end-REE), Case1_sim.u(:,1:end-REE),'k--');
% % plot(Case2_sim.time(:,1:end-REE),   Case2_sim.u(:,1:end-REE),'b-', 'Marker', 'o', 'LineWidth', 1, ...
% %     'MarkerIndices', 1:case2_marker:length(Case1_sim.time), 'MarkerSize', mark_size);
% % plot(Case3_sim.time(:,1:end-REE), Case3_sim.u(:,1:end-REE),'Color', case3_color, 'LineStyle','-.');
% % xlabel("Time (s)",'Interpreter','latex'); ylabel("$u(t)$",'Interpreter','latex');
% % xlim([0 5])
% % 
% % subplot(1,3,2)
% % hold on; grid on; box on;
% % set(gca, 'GridLineStyle', ':');
% % plot(Case1_sim.x(1,1:end-REE), Case1_sim.x(2,1:end-REE),'k--');
% % plot(Case2_sim.x(1,1:end-REE), Case2_sim.x(2,1:end-REE),'b-','Marker', 'o', 'LineWidth', 1, ...
% %     'MarkerIndices', 1:case2_marker:length(Case1_sim.time), 'MarkerSize', mark_size);
% % plot(Case3_sim.x(1,1:end-REE), Case3_sim.x(2,1:end-REE),'Color', case3_color, 'LineStyle','-.');
% % plot(Case1_sim.ref(1,1:end-REE), Case1_sim.ref(2,1:end-REE),'g');
% % plot(bx, by,'r-');
% % xlabel('$x_1$', 'Interpreter','latex'); ylabel('$x_2$', 'Interpreter','latex');
% % xlim([-1.5 1.5])
% % ylim([-2 2])
% % legend({'Unsafe', ...
% %         'Safe', ...
% %         'Shielded', ...
% %         '$r(t)$', ...
% %         '$h(x)=0$'},...
% %         'Interpreter','latex');
% % 
% % subplot(1,3,3)
% % hold on; grid on; box on;
% % set(gca, 'GridLineStyle', ':');
% % plot(Case1_sim.time(:,1:end-REE), Case1_sim.h_log(:,1:end-REE),'k--');
% % plot(Case2_sim.time(:,1:end-REE),   Case2_sim.h_log(:,1:end-REE),'b-','Marker', 'o', 'LineWidth', 1, ...
% %     'MarkerIndices', 1:case2_marker:length(Case1_sim.time), 'MarkerSize', mark_size);
% % plot(Case3_sim.time(:,1:end-REE), Case3_sim.h_log(:,1:end-REE),'Color', case3_color, 'LineStyle','-.'); 
% % xlabel("Time (s)"); ylabel("error");
% % yline(0,'r-');
% % xlabel("Time (s)", 'Interpreter','latex');
% % ylabel("$h(x)$",   'Interpreter','latex');
% % xlim([0 5])
% 
% %%%% 4x1 plot for breakout
% % figure; 
% % subplot(1,4,1)
% % hold on; grid on; box on;
% % set(gca, 'GridLineStyle', ':');
% % plot(Case1_sim.time(:,1:end-REE), Case1_sim.u(:,1:end-REE),'k--');
% % plot(Case2_sim.time(:,1:end-REE),   Case2_sim.u(:,1:end-REE),'b-', 'Marker', 'o', 'LineWidth', 1, ...
% %     'MarkerIndices', 1:case2_marker:length(Case1_sim.time), 'MarkerSize', mark_size);
% % plot(Case3_sim.time(:,1:end-REE), Case3_sim.u(:,1:end-REE),'Color', case3_color, 'LineStyle','-.');
% % xlabel("Time (s)",'Interpreter','latex'); ylabel("$u(t)$",'Interpreter','latex');
% % xlim([0 5])
% % 
% % subplot(1,4,2)
% % hold on; grid on; box on;
% % set(gca, 'GridLineStyle', ':');
% % plot(Case1_sim.x(1,1:end-REE), Case1_sim.x(2,1:end-REE),'k--');
% % plot(Case2_sim.x(1,1:end-REE), Case2_sim.x(2,1:end-REE),'b-','Marker', 'o', 'LineWidth', 1, ...
% %     'MarkerIndices', 1:case2_marker:length(Case1_sim.time), 'MarkerSize', mark_size);
% % plot(Case3_sim.x(1,1:end-REE), Case3_sim.x(2,1:end-REE),'Color', case3_color, 'LineStyle','-.');
% % plot(Case1_sim.ref(1,1:end-REE), Case1_sim.ref(2,1:end-REE),'g');
% % plot(bx, by,'r-');
% % xlabel('$x_1$', 'Interpreter','latex'); ylabel('$x_2$', 'Interpreter','latex');
% % xlim([-1.5 1.5])
% % ylim([-2 2])
% % legend({'Unsafe', ...
% %         'Safe', ...
% %         'Shielded', ...
% %         '$r(t)$', ...
% %         '$h(x)=0$'},...
% %         'Interpreter','latex');
% % 
% % subplot(1,4,3)
% % hold on; grid on; box on;
% % set(gca, 'GridLineStyle', ':');
% % plot(Case1_sim.x(1,1:end-REE), Case1_sim.x(2,1:end-REE),'k--');
% % plot(Case2_sim.x(1,1:end-REE), Case2_sim.x(2,1:end-REE),'b-','Marker', 'o', 'LineWidth', 1, ...
% %     'MarkerIndices', 1:case2_marker:length(Case1_sim.time), 'MarkerSize', mark_size);
% % plot(Case3_sim.x(1,1:end-REE), Case3_sim.x(2,1:end-REE),'Color', case3_color, 'LineStyle','-.');
% % plot(Case1_sim.ref(1,1:end-REE), Case1_sim.ref(2,1:end-REE),'g');
% % plot(bx, by,'r-');
% % xlabel('$x_1$', 'Interpreter','latex'); ylabel('$x_2$', 'Interpreter','latex');
% % xlim([-1 .2])
% % ylim([-1.7 -.4])
% % 
% % 
% % subplot(1,4,4)
% % hold on; grid on; box on;
% % set(gca, 'GridLineStyle', ':');
% % plot(Case1_sim.time(:,1:end-REE), Case1_sim.h_log(:,1:end-REE),'k--');
% % plot(Case2_sim.time(:,1:end-REE),   Case2_sim.h_log(:,1:end-REE),'b-','Marker', 'o', 'LineWidth', 1, ...
% %     'MarkerIndices', 1:case2_marker:length(Case1_sim.time), 'MarkerSize', mark_size);
% % plot(Case3_sim.time(:,1:end-REE), Case3_sim.h_log(:,1:end-REE),'Color', case3_color, 'LineStyle','-.'); 
% % xlabel("Time (s)"); ylabel("error");
% % yline(0,'r-');
% % xlabel("Time (s)", 'Interpreter','latex');
% % ylabel("$h(x)$",   'Interpreter','latex');
% % xlim([0 5])
% 
% 
% 

clear; clc; close all;
load("SNAC_safe_model_based_1.mat")

% saveFigures("model_based_barrier1_run1")

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

case3_color = [1 0.5 0];
%% =======================
%  Plots
% =======================
REE = 500;
figure; hold on; grid on;
plot(Case1_sim.time(:,1:end-REE), Case1_sim.h_log(:,1:end-REE),'k--');
plot(Case2_sim.time(:,1:end-REE),   Case2_sim.h_log(:,1:end-REE),'b-');
plot(Case3_sim.time(:,1:end-REE), Case3_sim.h_log(:,1:end-REE),'Color', case3_color, 'LineStyle','-.','LineWidth',1.5); 
xlabel("Time (s)"); ylabel("error");
yline(0,'r:','LineWidth',1.0);
xlabel("Time (s)", 'Interpreter','latex');
ylabel("$h(x)$",   'Interpreter','latex');


figure; hold on; grid on;
plot(Case1_sim.x(1,1:end-REE), Case1_sim.x(2,1:end-REE),'k--');
plot(Case2_sim.x(1,1:end-REE), Case2_sim.x(2,1:end-REE),'b-');
plot(Case3_sim.x(1,1:end-REE), Case3_sim.x(2,1:end-REE),'Color', case3_color, 'LineStyle','-.','LineWidth',1.5);
plot(Case1_sim.ref(1,1:end-REE), Case1_sim.ref(2,1:end-REE),'g');
plot(bx, by,'r:', 'LineWidth', 2);
xlabel('$x_1$', 'Interpreter','latex'); ylabel('$x_2$', 'Interpreter','latex');
xlim([-1.5 1.5])
ylim([-2 2])
legend({'Unsafe', ...
        'Safe', ...
        'Shielded', ...
        '$r(t)$', ...
        '$h(x)=0$'},...
        'Interpreter','latex');

figure; hold on; grid on;
plot(Case1_sim.time(:,1:end-REE), Case1_sim.u(:,1:end-REE),'k--');
plot(Case2_sim.time(:,1:end-REE),   Case2_sim.u(:,1:end-REE),'b-');
plot(Case3_sim.time(:,1:end-REE), Case3_sim.u(:,1:end-REE),'Color', case3_color, 'LineStyle','-.', 'LineStyle',':','LineWidth',1.5);
xlabel("Time (s)",'Interpreter','latex'); ylabel("u(t)",'Interpreter','latex');


%% =======================
%  Plots
% =======================
REE = 500;
figure; 
subplot(1,3,1)
hold on; grid on;
plot(Case1_sim.time(:,1:end-REE), Case1_sim.h_log(:,1:end-REE),'k--');
plot(Case2_sim.time(:,1:end-REE),   Case2_sim.h_log(:,1:end-REE),'b-');
plot(Case3_sim.time(:,1:end-REE), Case3_sim.h_log(:,1:end-REE),'Color', case3_color, 'LineStyle','-.','LineWidth',1.5); 
xlabel("Time (s)"); ylabel("error");
yline(0,'r:','LineWidth',1.0);
xlabel("Time (s)", 'Interpreter','latex');
ylabel("$h(x)$",   'Interpreter','latex');
xlim([0 5])

subplot(1,3,2)
hold on; grid on;
plot(Case1_sim.time(:,1:end-REE), Case1_sim.u(:,1:end-REE),'k--');
plot(Case2_sim.time(:,1:end-REE),   Case2_sim.u(:,1:end-REE),'b-');
plot(Case3_sim.time(:,1:end-REE), Case3_sim.u(:,1:end-REE),'Color', case3_color, 'LineStyle','-.', 'LineStyle',':','LineWidth',1.5);
xlabel("Time (s)",'Interpreter','latex'); ylabel("u(t)",'Interpreter','latex');
xlim([0 5])

subplot(1,3,3)
hold on; grid on;
plot(Case1_sim.x(1,1:end-REE), Case1_sim.x(2,1:end-REE),'k--');
plot(Case2_sim.x(1,1:end-REE), Case2_sim.x(2,1:end-REE),'b-');
plot(Case3_sim.x(1,1:end-REE), Case3_sim.x(2,1:end-REE),'Color', case3_color, 'LineStyle','-.','LineWidth',1.5);
plot(Case1_sim.ref(1,1:end-REE), Case1_sim.ref(2,1:end-REE),'g');
plot(bx, by,'r:', 'LineWidth', 1.0);
xlabel('$x_1$', 'Interpreter','latex'); ylabel('$x_2$', 'Interpreter','latex');
xlim([-1.5 1.5])
ylim([-2 2])
legend({'Unsafe', ...
        'Safe', ...
        'Shielded', ...
        '$r(t)$', ...
        '$h(x)=0$'},...
        'Interpreter','latex');

% legend({'Unsafe policy', ...
%         'Safe exploration', ...
%         'Shielded unsafe policy', ...
%         '$r(t)$', ...
%         '$h(x) = 0$'}, ...
%         'Location','northoutside', ...
%         'Orientation','horizontal', ...
%         'NumColumns',5, ...
%         'Interpreter','latex');




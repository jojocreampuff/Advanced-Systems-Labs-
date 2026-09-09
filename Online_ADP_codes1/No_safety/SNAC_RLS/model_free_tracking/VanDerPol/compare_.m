clear; close all; clc;
load("RLS_SNAC_MF_GP.mat") % this is alpha = 1
W = RLS_SNAC_MF.W_final;
R = 1;
Q = diag([100, 10]);
% figure;
% plot(TIME,error_history(:,1:2)');
% hold on; grid on; box on;
% set(gca, 'GridLineStyle', ':');
% % ylim([-.03 .03])
% legend("$e_{1}$","$e_{2}$", 'Interpreter', 'latex')
% ylabel("Tracking Errors", 'Interpreter', 'latex')
% xlabel ('Time (s)', 'Interpreter', 'latex');
% 
% figure;
% subplot(2,1,1)
% plot(TIME,error_history(:,1)');
% hold on; grid on; box on;
% set(gca, 'GridLineStyle', ':');
% ylabel("$e_1$", 'Interpreter', 'latex')
% xlabel ('Time (s)', 'Interpreter', 'latex');
% subplot(2,1,2)
% plot(TIME,error_history(:,2)');
% hold on; grid on; box on;
% set(gca, 'GridLineStyle', ':');
% ax = gca; ax.Box = 'on'; ax.LineWidth = .5; ax.XColor = 'k'; ax.YColor = 'k';
% ylabel("$e_2$", 'Interpreter', 'latex')
% xlabel ('Time (s)', 'Interpreter', 'latex');
% 
% figure;
% plot(TIME,W_history);
% hold on; grid on; box on;
% set(gca, 'GridLineStyle', ':');
% ax = gca; ax.Box = 'on'; ax.LineWidth = .5; ax.XColor = 'k'; ax.YColor = 'k';
% ylabel("$\widehat{W}$", 'Interpreter', 'latex')
% xlabel ('Time (s)', 'Interpreter', 'latex');

% dynamics non-linear
f = @(x) [x(2);
    (1-x(1).^2).*x(2) - x(1)];
g = [ 0 ;  1 ];

T = 10;
dt = 0.01;
N = T/dt;
time = dt:dt:T;
x_online = zeros(2,N);
x_online(:,1) = [0; 0];
u_online = zeros(1,N);
reference = zeros(2,N);

for i = 1:N
    T = dt*i;
    [reference(:,i), ref_dot] = ref_refdot(T);

    phi = PHI(x_online(:,i) - reference(:,i),  reference(:,i));
    G_z = [g; 0*g];
    u_online(:,i) = -0.5*R^-1 * G_z' * W'*phi;
    u_online(:,i) = max(min(u_online(:,i), 10), -10);

    x_online(:, i+1) = x_online(:,i) + dt *( f(x_online(:,i)) + g* u_online(:,i));
end

load("DP_vanderpol_tracking.mat", "u_DP","x_DP")
% x_DP = x_DP2;
% u_DP = u_DP2;
cut = 500;
figure; 
hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(time(1:end-cut), x_online(1,1:end-cut-1 ), '--r *', 'LineWidth', 0.9, 'MarkerIndices',1:100:length(time), "MarkerSize",5)
plot(time(1:end-cut), x_DP(1,1:end-cut-1 ), '-.ks', 'LineWidth', 0.9, 'MarkerIndices',1:120:length(time),"MarkerSize",5)
plot(time(1:end-cut), reference(1, 1:end-cut ),"Color","g","LineStyle","-","LineWidth",1)
xlabel("Time (s)", 'Interpreter', 'latex')
ylabel("$x_{1}$", 'Interpreter', 'latex')
legend("Online SNAC","DP", "$r$", 'Interpreter', 'latex')


figure;
hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(time(1:end-cut), x_online(2,1:end-cut -1), '--r *', 'LineWidth', 0.9, 'MarkerIndices',1:100:length(time), "MarkerSize",5)
plot(time(1:end-cut), x_DP(2,1:1:end-cut -1), '-.ks', 'LineWidth', 0.9, 'MarkerIndices',1:120:length(time),"MarkerSize",5)
plot(time(1:end-cut), reference(2,1:end-cut ),"Color","g","LineStyle","-","LineWidth",1)
xlabel("Time (s)", 'Interpreter', 'latex')
ylabel("$x_{2}$", 'Interpreter', 'latex')
legend("Online SNAC","DP", "$\dot{r}$", 'Interpreter', 'latex')


figure;
hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(x_online(1,1:end-cut ), x_online(2,1:end-cut ), '--r *', 'LineWidth', 0.9, 'MarkerIndices',1:100:length(x_online(2,1:end-cut )), "MarkerSize",5)
plot(x_DP(1,1:end-cut ), x_DP(2,1:end-cut), '-.ks', 'LineWidth', 0.9, 'MarkerIndices',1:150:length(x_online(2,1:end-cut )),"MarkerSize",5)
plot(reference(1,1:end-cut+1), reference(2,1:end-cut+1),"Color","g","LineStyle","-","LineWidth",1)
xlabel("$x_{1}$", 'Interpreter', 'latex')
ylabel("$x_{2}$", 'Interpreter', 'latex')
legend("Online SNAC","DP", "reference", 'Interpreter', 'latex')

figure;
hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(time(1:end-cut),u_online(1:end-cut), '--r *', 'LineWidth', 0.9, 'MarkerIndices',1:100:length(u_online(1:end-cut)), "MarkerSize",5)
plot(time(1:end-cut), u_DP(1:end-cut), '-.ks', 'LineWidth', 0.9, 'MarkerIndices',1:150:length(u_online(1:end-cut)),"MarkerSize",5)
xlabel("Time (s)", 'Interpreter', 'latex')
ylabel("Control Magnitude", 'Interpreter', 'latex')
legend("Online SNAC", "DP", 'Interpreter', 'latex')

osnac_DP_state_mae = mae(x_online - x_DP)
osnac_DP_control_mae = mae(u_online - u_DP)
err_online = x_online(:,1:end-1) - reference;
err_dp = x_DP(:,1:end-1) - reference;
snac_tracking_cost = dt * sum( sum( err_online .* (Q*err_online), 1 ) )
snac_control_cost = dt * sum( sum( u_online .* (R*u_online), 1 ) )
snac_cost = snac_tracking_cost + snac_control_cost;

dp_tracking_cost = dt * sum( sum( err_dp .* (Q*err_dp), 1 ) )
dp_control_cost = dt * sum( sum( u_DP .* (R*u_DP), 1 ) )
dp_cost = dp_tracking_cost + dp_control_cost;
% save("online_snac_tracking_2.mat")

% saveFigures("p3_online_snac_tracking_thesis_vectors_render")


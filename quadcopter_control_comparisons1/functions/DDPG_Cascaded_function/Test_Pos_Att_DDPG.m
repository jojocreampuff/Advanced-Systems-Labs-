clear; clc; close all;

rng(5,'twister'); 

Ts_Sim = 0.01;  % best time step 0.01
Tf = 50;
m = 1; g = 9.81;

mdl = 'Sim_PA_DDPG';
open_system(mdl)

load("Position.mat", "agent");
agent.SampleTime = Ts_Sim;
load("Attitude.mat", "agent2");
agent2.SampleTime = Ts_Sim;

state_noise = 0.05;
control_noise = 0.05;
W_nominal = 5;
Control_Saturation = 1;  % 1 mean on saturation

T = (0:Ts_Sim:Tf-Ts_Sim)';     % Nx1 column time (or start at 0, see note below)
N = length(T);
Wxyz = gen_wind_vector(W_nominal, Ts_Sim, Tf);  % should be 3xN

u = Wxyz.';                  % Nx3

Wxyz_timeseries = [T u];     % Nx4  (time + 3 signals)

% Initial Condition
%% Initial Condition
% IC = zeros(12,1); % easy

IC = [5; 5; 0; zeros(9,1)]; % moderate

% IC = [5; 5; 0; -1.16; -5.10; 0.79; 0.13; -0.54; pi/2; -0.071; -0.0025; 0.51]; % hard
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
simOut = sim('Sim_PA_DDPG');
DDPG_sim_time = toc;

disp(['Simulation runtime: ', num2str(DDPG_sim_time), ' seconds']) % 17.9222 seconds

X = squeeze(simOut.X);
U = squeeze(simOut.U);
U_pos = squeeze(simOut.U_pos);
X_ref = simOut.X_ref';
pos_error = simOut.pos_error';
att_error = simOut.att_error';

Q_pos = diag([1,1,1,1,1,1])*1; % continous time parameters (PLEASE CHECK IF YOUR WORKING IN DISCRETE OR CONT)
R_pos = diag([1,1,1])*1;
Q_att = diag([1,1,1,1,1,1])*1;
R_att = diag([1,1,1])*1;

pos_tracking_cost = Ts_Sim * sum( sum( pos_error .* (Q_pos*pos_error), 1 ) );
pos_control_cost = Ts_Sim  * sum( sum( U_pos .* (R_pos*U_pos), 1 ) );

att_tracking_cost = Ts_Sim * sum( sum( att_error .* (Q_att*att_error), 1 ) );
att_control_cost = Ts_Sim  * sum( sum( U(2:4,:) .* (R_att*U(2:4,:)), 1 ) );

DDPG_results.DDPG_c_cost_total = pos_tracking_cost + pos_control_cost +att_control_cost + att_tracking_cost;

% save all useful info in a struck
DDPG_results.X = X;
DDPG_results.U = U;
DDPG_results.X_ref = X_ref;
DDPG_results.error = [pos_error;att_error];
DDPG_results.simtime = DDPG_sim_time;
DDPG_error = DDPG_results.error;
DDPG_tracking_error = (DDPG_error(1,:).^2 + DDPG_error(2,:).^2 + DDPG_error(3,:).^2).^0.5;
DDPG_results.DDPG_tracking_error = DDPG_tracking_error;
DDPG_results.DDPG_tracking_error_norm = Ts_Sim*sum(DDPG_tracking_error,2);

save('DDPG_simulation.mat','DDPG_results', 'Ts_Sim', 'state_noise', 'control_noise', 'W_nominal', 'Control_Saturation');

% disp("======= DDPG Results ======")
fprintf("DDPG simulation time: %.4f\n", DDPG_results.simtime);
fprintf("DDPG total tracking error: %.4f\n", DDPG_results.DDPG_tracking_error_norm);
fprintf("DDPG tracking cost: %.4f\n", DDPG_results.DDPG_c_cost_total(end));

figure;
hold on;
plot(T(1:end-1), DDPG_results.DDPG_tracking_error(1,1:N-1), 'k', 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5);
ylabel('Tracking Error (m)', 'Interpreter', 'latex');
% ylim([0 1])
title('L2 Norm Tracking Error', 'Interpreter', 'latex');
% legend(["DDPG","SMC","FL","Offline AC","Offline SNAC","Online AC","Online SNAC","Online Single Critic"], 'Interpreter', 'latex');
grid on;

figure;
grid on
hold on
refxyz = DDPG_results.X_ref(1:6,:);
plot3(refxyz(1,:), refxyz(2,:), -refxyz(3,:), '-', 'Color', [0 1 0], 'Linewidth', .9)
% DDPG
plot3(DDPG_results.X(1,:), DDPG_results.X(2,:), -DDPG_results.X(3,:), 'k', 'LineWidth', 0.9, 'Marker', 'v', 'LineWidth', 0.9, ...
    'MarkerIndices', 1:1000:length(T), 'MarkerSize', 5)
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')

% pqr_Limit = 10;       % Attitude Train Limit
% 
% phi_limit = pi/10;    % Attitude Train Limit
% theta_limit = pi/10;  % Attitude Train Limit
% psi_limit = pi/10; 


% Best Solution
% agent.SampleTime=0.001;
% agent2.SampleTime=0.001;
% 
% save("Position.mat", "agent");
% save("Attitude.mat", "agent2");

% load('out.mat')
% 
% % === Extract Data ===
% t = out.tout;
% rx = out.Ref.signals.values(:,1);
% ry = out.Ref.signals.values(:,2);
% rz = -1*out.Ref.signals.values(:,3);
% 
% x = out.Actual.signals.values(:,1);
% y = out.Actual.signals.values(:,2);
% z = -1*out.Actual.signals.values(:,3);
% 
% ex = out.Error.signals.values(:,1);
% ey = out.Error.signals.values(:,2);
% ez = out.Error.signals.values(:,3);
% 
% exdot = out.Error.signals.values(:,4);
% eydot = out.Error.signals.values(:,5);
% ezdot = out.Error.signals.values(:,6);
% 
% rms_xyzdot = rms(exdot.^2 + eydot.^2 + ezdot.^2); 
% 
% 
% % === 3D plot Ref Actual ===
% figure
% plot3(rx, ry, rz, 'b', 'LineWidth', 2)
% hold on
% plot3(x, y, z, 'r--', 'LineWidth', 2)
% hold off
% 
% grid on; box on
% xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)')
% xlim([-4 4]); ylim([-4 4]); zlim([0 50])
% title('3D Trajectory')
% view(3)
% legend('Reference','Actual','Location','best')
% 
% 
% % === Plot Each Axis Error ===
% % X axis
% figure
% plot(t, rx,'b', t,x,'r--', t,ex,'k:','LineWidth',1.5)
% grid on
% xlabel('Time (sec)'); ylabel('X (m)');
% legend('Reference X ','Actual X','Error')
% title('X Axis Tracking')
% 
% % Y axis
% figure
% plot(t, ry,'b', t,y,'r--', t,ey,'k:','LineWidth',1.5)
% grid on
% xlabel('Time (sec)'); ylabel('Y (m)');
% legend('Reference Y ','Actual Y','Error')
% title('Y Axis Tracking')
% 
% % Z axis
% figure
% plot(t, rz,'b', t,z,'r--', t,ez,'k:','LineWidth',1.5)
% grid on
% xlabel('Time (sec)'); ylabel('Z (m)');
% legend('Reference Z ','Actual Z','Error')
% title('Z Axis Tracking')
% 
% 
% % ==== Error Percentage   =====
% rms_x  = rms(ex) / rms(rx) * 100;
% rms_y  = rms(ey) / rms(ry) * 100;
% rms_z  = rms(ez) / rms(rz) * 100;
% 
% fprintf('Axis %% error (RMS%%): X=%.2f%%, Y=%.2f%%, Z=%.2f%%\n', rms_x, rms_y, rms_z);
% 
% % 3D Error Percentage
% e3 = sqrt(ex.^2 + ey.^2 + ez.^2);                 % position error magnitude
% r3 = sqrt(rx.^2 + ry.^2 + rz.^2);                 % reference position magnitude
% rms_3d  = rms(e3) / rms(r3) * 100;   % RMS% in 3D
% 
% fprintf('3D %% error: RMS%%=%.2f%%\n', rms_3d);
% 
% % Axis % error (RMS%): X=0.91%, Y=1.05%, Z=0.11%
% % 3D % error: RMS%=0.14%
% 
% agent2.SampleTime = 0.01;



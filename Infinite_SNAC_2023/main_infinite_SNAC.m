clc; clear; close all;
N = 50000;
% Add paths
addpath('Position Controller')
addpath('Attitude Controller')
addpath('functions')

% Load Albertos workspaces
% load('infinite_Position_SNAC_workspace2.mat','Position_W','Position_R','Position_F','Position_G','dt','grav')
% load('infinite_Attitude_SNAC_workspace2.mat','Attitude_W','Attitude_R','Attitude_F','Attitude_G')

load('test_workspace_pos.mat','Position_W','Position_R','Position_F','Position_G','dt','grav')
load('test_workspace_att_V.mat','Attitude_W','Attitude_R','Attitude_F','Attitude_G')
% load('test_workspace_att.mat','Attitude_W','Attitude_R','Attitude_F','Attitude_G')

% Load necessary variables for the position and attitude
Position.Position_W = Position_W;   % NN weights
Position.Position_G = Position_G;   % Position control dynamics
Position.Position_R = Position_R;   % Control penalizing matrixPosistionF_tx_

Attitude.Attitude_W = Attitude_W;   % NN weights
Attitude.Attitude_G = Attitude_G;   % Attitude control dynamics
Attitude.Attitude_R = Attitude_R;   % Control penalizing matrix

% Define simulation parameters
parameters.dt   = 0.004;    % time step
parameters.t_f  = 100;       % final time
parameters.grav = 9.81;     % gravity (m/s^2)
parameters.m    = 3.2;        % mass (kg)
parameters.Ix   = 2;      % moments of inertia (kg*m^2)
parameters.Iy   = 2;      %
parameters.Iz   = 4;      %

% Define desired reference as function of time
reference = @(t)...
            [(1-exp(-0.01*t))*9.81*cos(0.2*t);  % reference_x
             (1-exp(-0.01*t))*9.81*sin(0.2*t);  % reference_y
             -.5*t];                             % reference_z

% References can be waypoints (not function of time)
% reference = @(t)...
%             [9.81*cos(0.2*t);       % reference_x
%              9.81*sin(0.2*t);     % reference_y
%              -4*cos(1*t)-9.81];      % reference_z

% References can be waypoints (not function of time)
% reference = @(t)...
%             [3;       % reference_x
%              2.5;     % reference_y
%              -5];      % reference_z

% Define initial condition, each column is a new set of ICs
IC = [0 5 -5  5 -5; % x
      0 5 -5 -5  5; % y
      0 0  0  0  0; % z
      zeros(9,5)];  % velocity, angles, angular velocities

% % Define initial condition, each column is a new set of ICs
% IC = [0 5 -5; % x
%       0 5 -5; % y
%       0 -1  -2; % z
%       zeros(9,3)];  % velocity, angles, angular velocities

% One set of IC
% IC = [0; 
%       0; 
%       0; 
%       zeros(9,1)];

noise = 0; % 600% noise

% Simulating for all IC, simulations saved in structures
for i = 1:size(IC,2)
    results = simulate(Position, Attitude, parameters, reference, IC(:,i),noise); %max 6 
    simulations.(['results_', num2str(i)]) = results;
    x.(['x_',[num2str(i)]]) = results.x;
    u.(['u_',[num2str(i)]]) = results.u;
    uxyz.(['uxyz_',[num2str(i)]]) = results.uxyz;
    Pos_error.(['Pos_error_',[num2str(i)]]) = results.Pos_error;
    Att_error.(['Att_error_',[num2str(i)]]) = results.Att_error;
    r_smooth.(['r_smooth_',[num2str(i)]]) = results.r_smooth;
    r_initial.(['r_initial_',[num2str(i)]]) = results.r_initial;
    angles_ref.(['angles_ref_',[num2str(i)]]) = results.angles_ref;
    PWM_channels.(['PWM_channels_',[num2str(i)]]) = results.PWM_channels;
    each_motor_thrust.(['each_motor_thrust_',[num2str(i)]]) = results.each_motor_thrust;
    time = results.time;
end

%% Plotting 
figure(1)
plot3(r_initial.r_initial_1(1,:), r_initial.r_initial_1(2,:), -r_initial.r_initial_1(3,:), 'b--', 'Linewidth', 1.5)
grid on
hold on
for i = 1:size(IC,2)
plot3(x.(['x_',[num2str(i)]])(1,:), x.(['x_',[num2str(i)]])(2,:), -x.(['x_',[num2str(i)]])(3,:), 'Linewidth', 1.5)
end
plot3(r_initial.r_initial_1(1,:), r_initial.r_initial_1(2,:), -r_initial.r_initial_1(3,:), 'b--', 'Linewidth', 1.5)
title('3D Trajectory')
xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
legend('Reference Trajectory', 'Path 1','Path 2','Path 3','path 4','path 5', 'Location', 'northeast');

c_size = length(u.u_1(1,:))-1;
figure(2)
hold on
grid on
plot(time(1:c_size), u.u_1(1,1:c_size), 'Linewidth', 1.5)
title('Quadcopter Controls')
ylabel('$f_t$ (N)','Interpreter','latex'), xlabel('time (s)')
% ylim([9.7 10.1])
% 
figure(3)
hold on
grid on
plot(time(1:c_size), u.u_1(2,1:c_size), 'Linewidth', 1.5)
ylabel('$\tau_x$ (Nm)','Interpreter','latex'), xlabel('time (s)')
% ylim([-4E-4 4E-4])
% 
figure(4)
hold on
grid on
plot(time(1:c_size), u.u_1(3,1:c_size), 'Linewidth', 1.5)
ylabel('$\tau_y$ (Nm)','Interpreter','latex'), xlabel('time (s)')
% ylim([-4E-4 8E-4])
% 
figure(5)
hold on
grid on
plot(time(1:c_size), u.u_1(4,1:c_size), 'Linewidth', 1.5)
ylabel('$\tau_z$ (Nm)','Interpreter','latex'), xlabel('time (s)')

%% Position Controller Output 
figure(6)
subplot(3,1,1)
title("Position Controller Output",'Interpreter','latex')
hold on
grid on
plot(time(1:c_size), uxyz.uxyz_1(1,1:c_size), 'Linewidth', 1.5)
ylabel('$u_x$ (m/s)','Interpreter','latex'), xlabel('time (s)')

subplot(3,1,2)
hold on
grid on
plot(time(1:c_size), uxyz.uxyz_1(2,1:c_size), 'Linewidth', 1.5)
ylabel('$u_y$ (m/s)','Interpreter','latex'), xlabel('time (s)')

subplot(3,1,3)
hold on
grid on
plot(time(1:c_size), uxyz.uxyz_1(3,1:c_size), 'Linewidth', 1.5)
ylabel('$u_z$ (m/s)','Interpreter','latex'), xlabel('time (s)')

%% Pos Error
figure(7)
subplot(3,1,1)
title("Position error",'Interpreter','latex')
hold on
grid on
plot(time(1:c_size), Pos_error.Pos_error_1(1,1:c_size), 'Linewidth', 1.5)
ylabel('$error_x$ (m)','Interpreter','latex'), xlabel('time (s)')

subplot(3,1,2)
hold on
grid on
plot(time(1:c_size), Pos_error.Pos_error_1(2,1:c_size), 'Linewidth', 1.5)
ylabel('$error_y$ (m)','Interpreter','latex'), xlabel('time (s)')

subplot(3,1,3)
hold on
grid on
plot(time(1:c_size), Pos_error.Pos_error_1(3,1:c_size), 'Linewidth', 1.5)
ylabel('$error_z$ (m)','Interpreter','latex'), xlabel('time (s)')


%% Att Error
figure(8)
subplot(3,1,1)
title("Attitude error",'Interpreter','latex')
hold on
grid on
plot(time(1:c_size), Att_error.Att_error_1(1,1:c_size), 'Linewidth', 1.5)
ylabel('$error_/phi$ (rad)','Interpreter','latex'), xlabel('time (s)')

subplot(3,1,2)
hold on
grid on
plot(time(1:c_size), Att_error.Att_error_1(2,1:c_size), 'Linewidth', 1.5)
ylabel('$error_/theta$ (rad)','Interpreter','latex'), xlabel('time (s)')

subplot(3,1,3)
hold on
grid on
plot(time(1:c_size), Att_error.Att_error_1(3,1:c_size), 'Linewidth', 1.5)
ylabel('$error_/psi$ (rad)','Interpreter','latex'), xlabel('time (s)')

%% Refernce angles given to att controller
figure(9)
subplot(6,1,1)
title("Ref Angles",'Interpreter','latex')
hold on
grid on
plot(time(1:c_size), angles_ref.angles_ref_1(1,1:c_size), 'Linewidth', 1.5)
ylabel('$/phi$ (rad)','Interpreter','latex'), xlabel('time (s)')

subplot(6,1,2)
hold on
grid on
plot(time(1:c_size), angles_ref.angles_ref_1(2,1:c_size), 'Linewidth', 1.5)
ylabel('$/theta$ (rad)','Interpreter','latex'), xlabel('time (s)')

subplot(6,1,3)
hold on
grid on
plot(time(1:c_size), angles_ref.angles_ref_1(3,1:c_size), 'Linewidth', 1.5)
ylabel('$/psi$ (rad)','Interpreter','latex'), xlabel('time (s)')

subplot(6,1,4)
hold on
grid on
plot(time(1:c_size), angles_ref.angles_ref_1(4,1:c_size), 'Linewidth', 1.5)
ylabel('$/phi_s$ (rad/s)','Interpreter','latex'), xlabel('time (s)')

subplot(6,1,5)
hold on
grid on
plot(time(1:c_size), angles_ref.angles_ref_1(5,1:c_size), 'Linewidth', 1.5)
ylabel('$/theta_s$ (rad/s)','Interpreter','latex'), xlabel('time (s)')

subplot(6,1,6)
hold on
grid on
plot(time(1:c_size), angles_ref.angles_ref_1(6,1:c_size), 'Linewidth', 1.5)
ylabel('$/psi_s$ (rad/s)','Interpreter','latex'), xlabel('time (s)')

%% PWM figure
figure(10)
subplot(4,1,1)
title("PWM Channels",'Interpreter','latex')
hold on
grid on
plot(time(1:c_size), PWM_channels.PWM_channels_1(1,1:c_size), 'Linewidth', 1.5)
ylabel('Ch1','Interpreter','latex'), xlabel('time (s)')

subplot(4,1,2)
hold on
grid on
plot(time(1:c_size), PWM_channels.PWM_channels_1(2,1:c_size), 'Linewidth', 1.5)
ylabel('ch2','Interpreter','latex'), xlabel('time (s)')

subplot(4,1,3)
hold on
grid on
plot(time(1:c_size), PWM_channels.PWM_channels_1(3,1:c_size), 'Linewidth', 1.5)
ylabel('ch3','Interpreter','latex'), xlabel('time (s)')

subplot(4,1,4)
hold on
grid on
plot(time(1:c_size), PWM_channels.PWM_channels_1(4,1:c_size), 'Linewidth', 1.5)
ylabel('ch4','Interpreter','latex'), xlabel('time (s)')

%% Each Motor Thrust
figure(11)
subplot(4,1,1)
title("Each Motor Thrust",'Interpreter','latex')
hold on
grid on
plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(1,1:c_size), 'Linewidth', 1.5)
ylabel('Motor 1 (N)','Interpreter','latex'), xlabel('time (s)')

subplot(4,1,2)
hold on
grid on
plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(2,1:c_size), 'Linewidth', 1.5)
ylabel('Motor 2 (N)','Interpreter','latex'), xlabel('time (s)')

subplot(4,1,3)
hold on
grid on
plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(3,1:c_size), 'Linewidth', 1.5)
ylabel('Motor 3 (N)','Interpreter','latex'), xlabel('time (s)')

subplot(4,1,4)
hold on
grid on
plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(4,1:c_size), 'Linewidth', 1.5)
ylabel('Motor 4 (N)','Interpreter','latex'), xlabel('time (s)')

% % Position Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure
% subplot(3,1,1)
% hold on
% grid on
% plot(time, r_initial.r_initial_1(1,1:length(time)),'b--', 'Linewidth', 1.5)
% for i = 1:3%size(IC,2)
% plot(time, x.(['x_',[num2str(i)]])(1,1:length(time)), 'Linewidth', 1.5)
% end
% plot(time, r_initial.r_initial_1(1,1:length(time)),'b--', 'Linewidth', 1.5)
% title('Position Tracking')
% ylabel('x (m)'), xlabel('time (s)')
% legend('Reference trajectory','Path 1','Path 2','Path 3','Location', 'northeast');
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(time, r_initial.r_initial_1(2,1:length(time)),'b--', 'Linewidth', 1.5)
% for i = 1:3%size(IC,2)
% plot(time, x.(['x_',[num2str(i)]])(2,1:length(time)), 'Linewidth', 1.5)
% end
% plot(time, r_initial.r_initial_1(2,1:length(time)),'b--', 'Linewidth', 1.5)
% ylabel('y (m)'), xlabel('time (s)')
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(time, -r_initial.r_initial_1(3,1:length(time)),'b--', 'Linewidth', 1.5)
% for i = 1:3%size(IC,2)
% plot(time, -x.(['x_',[num2str(i)]])(3,1:length(time)), 'Linewidth', 1.5)
% end
% plot(time, -r_initial.r_initial_1(3,1:length(time)),'b--', 'Linewidth', 1.5)
% ylabel('z (m)'), xlabel('time (s)')
% 
% % Control Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 
% % Velocity Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure
% subplot(3,1,1)
% hold on
% grid on
% plot(time, r_smooth.r_smooth_1(4,1:length(time)),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(4,1:length(time)), 'Linewidth', 1.5)
% plot(time, r_smooth.r_smooth_1(4,1:length(time)),'b--', 'Linewidth', 1.5)
% title('Velocity Tracking')
% ylabel('u (m/s)'), xlabel('time (s)')
% legend('Reference trajectory', 'Simulated','Location', 'northeast');
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(time, r_smooth.r_smooth_1(5,1:length(time)),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(5,1:length(time)), 'Linewidth', 1.5)
% plot(time, r_smooth.r_smooth_1(5,1:length(time)),'b--', 'Linewidth', 1.5)
% ylabel('v (m/s)'), xlabel('time (s)')
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(time, -r_smooth.r_smooth_1(6,1:length(time)),'b--', 'Linewidth', 1.5)
% plot(time, -x.x_1(6,1:length(time)), 'Linewidth', 1.5)
% plot(time, -r_smooth.r_smooth_1(6,1:length(time)),'b--', 'Linewidth', 1.5)
% ylabel('w (m/s)'), xlabel('time (s)')
% 
% % Angle Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% a_size = length(angles_ref.angles_ref_1(1,:))-1;
% figure
% subplot(3,1,1)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(1,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(7,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(1,1:a_size),'b--', 'Linewidth', 1.5)
% title('Angle Tracking')
% ylabel('$\phi$ (rad)','Interpreter','latex'), xlabel('time (s)')
% legend('Reference trajectory', 'Simulated','Location', 'northeast');
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(2,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(8,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(2,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('$\theta$ (rad)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(3,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(9,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(3,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('$\psi$ (rad)','Interpreter','latex'), xlabel('time (s)')
% ylim([-0.04 0.04])
% 
% % Angular Velocity Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure
% subplot(3,1,1)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(4,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(10,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(4,1:a_size),'b--', 'Linewidth', 1.5)
% title('Anglular Velocity Tracking')
% ylabel('p (rad/s)','Interpreter','latex'), xlabel('time (s)')
% legend('Reference trajectory', 'Simulated','Location', 'northeast');
% ylim([-5E-3 5E-3])
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(5,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(11,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(5,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('q (rad/s)','Interpreter','latex'), xlabel('time (s)')
% ylim([-5E-3 5E-3])
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(6,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(12,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(6,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('r (rad/s)','Interpreter','latex'), xlabel('time (s)')
% ylim([-5E-3 5E-3])
% 
% % Angular Velocity and Controls %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% figure
% sgtitle('Angular Velocity Stabilization')
% subplot(3,2,1)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(4,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(10,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(4,1:a_size),'b--', 'Linewidth', 1.5)
% title('Anglular Velocities')
% ylabel('p (rad/s)','Interpreter','latex'), xlabel('time (s)')
% ylim([-5E-2 5E-2]); 
% xlim([0 1])
% 
% subplot(3,2,3)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(5,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(11,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(5,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('q (rad/s)','Interpreter','latex'), xlabel('time (s)')
% ylim([-8E-2 8E-2]); 
% xlim([0 1])
% 
% subplot(3,2,5)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(6,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(12,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(6,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('r (rad/s)','Interpreter','latex'), xlabel('time (s)')
% legend('Reference trajectory', 'Simulated','Location', 'northeast');
% xlim([0 1.5])
% 
% subplot(3,2,2)
% hold on
% grid on
% title('Torque Controls')
% plot(time(1:length(u.u_1(1,:))), u.u_1(2,:), 'Linewidth', 1.5)
% ylabel('$\tau_x$ (Nm)','Interpreter','latex'), xlabel('time (s)')
% ylim([-0.2 0.02]); 
% xlim([0 1])
% 
% subplot(3,2,4)
% hold on
% grid on
% plot(time(1:length(u.u_1(1,:))), u.u_1(3,:), 'Linewidth', 1.5)
% ylabel('$\tau_y$ (Nm)','Interpreter','latex'), xlabel('time (s)')
% ylim([-0.35 6E-2]); 
% xlim([0 1])
% 
% subplot(3,2,6)
% hold on
% grid on
% plot(time(1:length(u.u_1(1,:))), u.u_1(4,:), 'Linewidth', 1.5)
% ylabel('$\tau_z$ (Nm)','Interpreter','latex'), xlabel('time (s)')
% ylim([-0.04 0.04]); 
% xlim([0 1])

% saveFigures
% save('SNAC_simulations_workspace')
% save("z_reference","r_smooth","r_initial","-v7.3")
clc; clear; close all;

% Add paths
addpath('Position Controller')
addpath('Attitude Controller')
addpath('functions')

load('SNAC_pos_NEW.mat','Position_W','Position_Q','Position_R','Position_F','Position_G','dt','grav')
load('SNAC_att_NEW.mat')

% Load necessary variables for the position and attitude
Position.Position_W = Position_W;   % NN weights
Position.Position_G = Position_G;   % Position control dynamics
Position.Position_R = Position_R;   % Control penalizing matrixPosistionF_tx_
Position.Position_Q = Position_Q;

Attitude.Attitude_W = Attitude_W;   % NN weights
Attitude.Attitude_G = Attitude_G;   % Attitude control dynamics
Attitude.Attitude_R = Attitude_R;   % Control penalizing matrix
Attitude.Attitude_Q = Attitude_Q;
% adding non-dim to the system
Attitude.Attitude_G_bar = Attitude_G_bar;
Attitude.Attitude_F_bar = Attitude_F_bar;
Attitude.Attitude_max_states = max_states;

% Define simulation parameters
parameters.dt   = 0.004;    % time step
parameters.t_f  = 50;       % final time
parameters.grav = 9.81;     % gravity (m/s^2)
% parameters.m    = .8;        % mass (kg)
% parameters.Ix   = .005;      % moments of inertia (kg*m^2)
% parameters.Iy   = .005;      %
% parameters.Iz   = .009;      %
parameters.m    = 1;        % mass (kg)
parameters.Ix   = 0.3;      % moments of inertia (kg*m^2)
parameters.Iy   = 0.4;      %
parameters.Iz   = 0.5;      %

% Define desired reference as function of time
% reference = @(t)...
%             [(1-exp(-0.01*t))*9.81*cos(0.2*t);  % reference_x
%              (1-exp(-0.01*t))*9.81*sin(0.2*t);  % reference_y
%              -.1*t];                             % reference_z


reference = @(t) [5*cos(0.13*t);       % reference_x
             5*sin(0.13*t);     % reference_y
             -2*cos(.13*t)-5];      % reference_z

reference = @(t) [5*cos(0.13*t);       % reference_x
             5*sin(0.13*t);     % reference_y
             -2*cos(4*.13*t)-5];      % reference_z
% 
% reference = @(t)...
%             [0;       % reference_x
%              5*sin(0.2*t);     % reference_y
%              -5*cos(0.2*t)-7];      % reference_z


% Define initial condition, each column is a new set of ICs
IC = [5 -5  5   -5;  % x
      5 -5 -5    5;  % y
      0  0  0    0;  % z
      0  0  0    0% u
      0  0  0   0% v
      0 0   0   0% w
      0 0   0   0% pitch
      0 0   0   0% roll
      1 0   0   0  % yaw
      0 0   0   0  
      0 0   0   0  
      0 0   0   0  ];      % velocity, angles, angular velocities

disturbance = 0; %
sensor_uncertanity = 0; % yes or no 
actuator_uncertianty = 0; % yes or no
noise = [disturbance, sensor_uncertanity, actuator_uncertianty];

% Simulating for all IC, simulations saved in structures
for i = 1:size(IC,2)
    results = simulate(Position, Attitude, parameters, reference, IC(:,i),noise); %max 6 
    % results = simulate_NN(Position, Attitude, parameters, reference, IC(:,i),noise, net); %max 6
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
    instant_cost_pos.(['instant_cost_pos_',[num2str(i)]]) = results.instant_cost_pos;
    instant_cost_att.(['instant_cost_att_',[num2str(i)]]) = results.instant_cost_att;
    cumulative_cost_pos.(['cumulative_cost_pos_',[num2str(i)]]) = results.cumulative_cost_pos;
    cumulative_cost_att.(['cumulative_cost_att_',[num2str(i)]]) = results.cumulative_cost_att;
    time = results.time;
end

c_size = length(u.u_1(1,:))-1;
%% 3D Plotting 
figure;
plot3(r_initial.r_initial_1(1,:), r_initial.r_initial_1(2,:), -r_initial.r_initial_1(3,:), 'b--', 'Linewidth', 1.5)
grid on
hold on
for i = 1:size(IC,2)
plot3(x.(['x_',[num2str(i)]])(1,:), x.(['x_',[num2str(i)]])(2,:), -x.(['x_',[num2str(i)]])(3,:), 'Linewidth', 1)
end
title('Tracking Trajectory 2', 'Interpreter', 'latex')
xlabel('x (m)', 'Interpreter', 'latex'), ylabel('y (m)', 'Interpreter', 'latex'), zlabel('z (m)', 'Interpreter', 'latex')
legend('$r(t)$', '$x_{0_1}$','$x_{0_2}$','$x_{0_3}$','$x_{0_4}$', 'Location', 'northeast', 'Interpreter', 'latex');
% zlim([0 5])
% saveFigures("trajectory_2")
save('SNAC_simulations_workspace.mat')


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
% % ylim([-0.04 0.04])

%% Att Error
% figure;
% subplot(3,1,1)
% title("Attitude error",'Interpreter','latex')
% hold on
% grid on
% plot(time(1:c_size), Att_error.Att_error_1(1,1:c_size), 'Linewidth', 1.5)
% ylabel('$e_/phi$ (rad)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(time(1:c_size), Att_error.Att_error_1(2,1:c_size), 'Linewidth', 1.5)
% ylabel('$e_/theta$ (rad)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(time(1:c_size), Att_error.Att_error_1(3,1:c_size), 'Linewidth', 1.5)
% ylabel('$e_/psi$ (rad)','Interpreter','latex'), xlabel('time (s)')

% Angular Velocity Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
% % ylim([-5E-3 5E-3])
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(5,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(11,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(5,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('q (rad/s)','Interpreter','latex'), xlabel('time (s)')
% % ylim([-5E-3 5E-3])
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(time(1:a_size), angles_ref.angles_ref_1(6,1:a_size),'b--', 'Linewidth', 1.5)
% plot(time, x.x_1(12,1:length(time)), 'Linewidth', 1.5)
% plot(time(1:a_size), angles_ref.angles_ref_1(6,1:a_size),'b--', 'Linewidth', 1.5)
% ylabel('r (rad/s)','Interpreter','latex'), xlabel('time (s)')
% % ylim([-5E-3 5E-3])

% Angular Velocity and Controls %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% %% PWM figure
% figure;
% subplot(4,1,1)
% title("PWM Channels",'Interpreter','latex')
% hold on
% grid on
% plot(time(1:c_size), PWM_channels.PWM_channels_1(1,1:c_size), 'Linewidth', 1.5)
% ylabel('Ch1','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(4,1,2)
% hold on
% grid on
% plot(time(1:c_size), PWM_channels.PWM_channels_1(2,1:c_size), 'Linewidth', 1.5)
% ylabel('Ch2','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(4,1,3)
% hold on
% grid on
% plot(time(1:c_size), PWM_channels.PWM_channels_1(3,1:c_size), 'Linewidth', 1.5)
% ylabel('ch3','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(4,1,4)
% hold on
% grid on
% plot(time(1:c_size), PWM_channels.PWM_channels_1(4,1:c_size), 'Linewidth', 1.5)
% ylabel('ch4','Interpreter','latex'), xlabel('time (s)')

% %% Each Motor Thrust
% figure;
% subplot(4,1,1)
% title("Actuator Thrust",'Interpreter','latex')
% hold on
% grid on
% plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(1,1:c_size), 'Linewidth', 1.5)
% ylabel('Motor 1 (N)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(4,1,2)
% hold on
% grid on
% plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(2,1:c_size), 'Linewidth', 1.5)
% ylabel('Motor 2 (N)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(4,1,3)
% hold on
% grid on
% plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(3,1:c_size), 'Linewidth', 1.5)
% ylabel('Motor 3 (N)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(4,1,4)
% hold on
% grid on
% plot(time(1:c_size), each_motor_thrust.each_motor_thrust_1(4,1:c_size), 'Linewidth', 1.5)
% ylabel('Motor 4 (N)','Interpreter','latex'), xlabel('time (s)')

% %% Position Controller Output 
% figure;
% subplot(3,1,1)
% title("Position Controller Output",'Interpreter','latex')
% hold on
% grid on
% plot(time(1:c_size), uxyz.uxyz_1(1,1:c_size), 'Linewidth', 1.5)
% ylabel('$u_x$ (m/s)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(3,1,2)
% hold on
% grid on
% plot(time(1:c_size), uxyz.uxyz_1(2,1:c_size), 'Linewidth', 1.5)
% ylabel('$u_y$ (m/s)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(3,1,3)
% hold on
% grid on
% plot(time(1:c_size), uxyz.uxyz_1(3,1:c_size), 'Linewidth', 1.5)
% ylabel('$u_z$ (m/s)','Interpreter','latex'), xlabel('time (s)')
% 
% %% Refernce angles given to att controller
% figure;
% subplot(6,1,1)
% title("Reference Angles",'Interpreter','latex')
% hold on
% grid on
% plot(time(1:c_size), angles_ref.angles_ref_1(1,1:c_size), 'Linewidth', 1.5)
% ylabel('$/phi$ (rad)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(6,1,2)
% hold on
% grid on
% plot(time(1:c_size), angles_ref.angles_ref_1(2,1:c_size), 'Linewidth', 1.5)
% ylabel('$/theta$ (rad)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(6,1,3)
% hold on
% grid on
% plot(time(1:c_size), angles_ref.angles_ref_1(3,1:c_size), 'Linewidth', 1.5)
% ylabel('$/psi$ (rad)','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(6,1,4)
% hold on
% grid on
% plot(time(1:c_size), angles_ref.angles_ref_1(4,1:c_size), 'Linewidth', 1.5)
% ylabel('$\dot{\phi}_b$','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(6,1,5)
% hold on
% grid on
% plot(time(1:c_size), angles_ref.angles_ref_1(5,1:c_size), 'Linewidth', 1.5)
% ylabel('$\dot{\theta}_b$','Interpreter','latex'), xlabel('time (s)')
% 
% subplot(6,1,6)
% hold on
% grid on
% plot(time(1:c_size), angles_ref.angles_ref_1(6,1:c_size), 'Linewidth', 1.5)
% ylabel('$\dot{\psi}_b$','Interpreter','latex'), xlabel('time (s)')





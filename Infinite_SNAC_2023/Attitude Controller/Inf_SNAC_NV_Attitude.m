clc; clear; close all;

% addpath("/home/users10/re606359/Desktop/matlab/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions")
% addpath("/home/users10/re606359/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions")
addpath('..\functions')

% Define plant dynamics
Ix = 0.005*1000;   % moment of inertia (g*m^2) 
Iy = 0.005*1000;  % moment of inertia (g*m^2)
Iz = 0.009*1000;   % moment of inertia (g*m^2)
% non-dimesnionalize these dynamics

% Define domains of training
PHI_max = pi/6; PHI_min = -pi/6;    % x1
THE_max = pi/6; THE_min = -pi/6;    % x2
PSI_max = pi; PSI_min = -pi;        % x3

p_max =  pi/8; p_min =  -pi/8;      % x4
q_max =  pi/8; q_min =  -pi/8;      % x5
r_max =  pi/10; r_min =  -pi/10;    % x6

x1_max = PHI_max;
x2_max = THE_max;
x3_max = PSI_max;
x4_max = p_max;
x5_max = q_max;
x6_max = r_max;
max_states = [x1_max; x2_max; x3_max; x4_max; x5_max; x6_max]*1.1;

u1_max = 1.3; u2_max = 1.3; u3_max = 1;
Umax = [u1_max; u2_max; u3_max]*1.5; % unit is Nm

% Define training Parameters
N_states =          6;
N_patterns =        1000;
max_training_loop = 15000;
threshold =         1e-3;
dt =                0.001;
discount = 1;
% Attitude_Q = dt*diag([1e6,1e6,1e4,1e5,1e5,1e4]);
Attitude_Q = diag([1,1,1,1,1,1])*1000;
Attitude_R = diag([1,1,1])*10;
% Attitude_R = dt*diag([0.5e4,0.5e4,0.5e4])


% Define simulation parameters
t_f = 6;
N_neurons = length(Basis_Func_84(ones(N_states,1)));
N = t_f/dt;   
Attitude_W = randn(N_neurons, N_states)*.01;
weight_plot = zeros((N_neurons* N_states),max_training_loop);

Attitude_f = @(x) [(x(4) + x(5)*(sin(x(1))*tan(x(2))) + x(6)*(cos(x(1))*tan(x(2))));
                   (x(5)*cos(x(1)) - x(6)*sin(x(1)));
                   (x(5)*sin(x(1))/cos(x(2)) + x(6)*cos(x(1))/cos(x(2)));
                   ((Iy - Iz) / Ix * x(5) * x(6));
                   ((Iz - Ix) / Iy * x(4) * x(6));
                   ((Ix - Iy) / Iz * x(4) * x(5))];
Attitude_g = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];

Attitude_f_bar = @(x_bar) [ (1/x1_max)*(x_bar(4)*x4_max + x_bar(5)*x4_max*(sin(x_bar(1)*x1_max)*tan(x_bar(2)*x2_max)) + x_bar(6)*x6_max*(cos(x_bar(1))*tan(x_bar(2))));
                            (1/x2_max)*(x_bar(5)*x5_max*cos(x_bar(1)*x1_max) - x_bar(6)*x6_max*sin(x_bar(1)*x1_max));
                            (1/x3_max)*(x_bar(5)*x5_max*sin(x_bar(1)*x1_max)/cos(x_bar(2)*x2_max) + x_bar(6)*x6_max*cos(x_bar(1)*x1_max)/cos(x_bar(2)*x2_max));
                            (1/x4_max)*((Iy - Iz) / Ix * x_bar(5) * x_bar(6)* x5_max* x6_max);
                            (1/x5_max)*((Iz - Ix) / Iy * x_bar(4) * x_bar(6)* x4_max* x6_max);
                            (1/x6_max)*((Ix - Iy) / Iz * x_bar(4) * x_bar(5)* x4_max* x5_max)      ];

Attitude_g_bar = [0 0 0; 0 0 0; 0 0 0; u1_max/(Ix*x4_max) 0 0; 0 u2_max/(Iy*x5_max) 0; 0 0 u3_max/(Iz*x6_max)];

% Euler integration
Attitude_F_bar = @(x_bar) x_bar + dt * Attitude_f_bar(x_bar);
Attitude_G_bar = @(x_bar) Attitude_g_bar * dt;

Attitude_F = @(x) x + dt * Attitude_f(x);
Attitude_G = @(x) Attitude_g * dt;

% Partial x_k+1 / partial x_k
% A = @(x)...
%     [
%     dt*(x(5)*cos(x(1))*tan(x(2)) - x(6)*sin(x(1))*tan(x(2))) + 1, dt*(x(6)*cos(x(1))*(tan(x(2))^2 + 1) + x(5)*sin(x(1))*(tan(x(2))^2 + 1)), 0, dt,  dt*sin(x(1))*tan(x(2)),  dt*cos(x(1))*tan(x(2)); 
%     -dt*(x(6)*cos(x(1)) + x(5)*sin(x(1))), 1, 0,  0,  dt*cos(x(1)),  -dt*sin(x(1)); 
%     dt*((x(5)*cos(x(1)))/cos(x(2)) - (x(6)*sin(x(1)))/cos(x(2))), dt*((x(6)*cos(x(1))*sin(x(2)))/cos(x(2))^2 + (x(5)*sin(x(1))*sin(x(2)))/cos(x(2))^2), 1,  0,  (dt*sin(x(1)))/cos(x(2)),  (dt*cos(x(1)))/cos(x(2)); 
%     0, 0, 0,  1,  (dt*x(6)*(Iy - Iz))/Ix,  (dt*x(5)*(Iy - Iz))/Ix; 
%      0, 0, 0,  -(dt*x(6)*(Ix - Iz))/Iy,  1,  -(dt*x(4)*(Ix - Iz))/Iy; 
%     0, 0, 0, (dt*x(5)*(Ix - Iy))/Iz,  (dt*x(4)*(Ix - Iy))/Iz,  1; 
%     ]; % row representation

tic
% Nonvectorzied SNAC training loop
for i = 1:max_training_loop
    basis_func = zeros(N_neurons, N_patterns); 
    lambda_k_plus_1_target = zeros(N_states, N_patterns); 
    % Generating target costate for all number of patterns
    for j = 1 : N_patterns
        X1 = PHI_min + (PHI_max - PHI_min) * rand;%(1, N_patterns);
        X2 = THE_min + (THE_max - THE_min) * rand;%(1, N_patterns);
        X3 = PSI_min + (PSI_max - PSI_min) * rand;%(1, N_patterns);    
        X4 = p_min + (p_max - p_min) * rand;%(1, N_patterns);
        X5 = q_min + (q_max - q_min) * rand;%(1, N_patterns);
        X6 = r_min + (r_max - r_min) * rand;%(1, N_patterns);

        % Random states within defined domain of trainig
        xbar_k = [X1/x1_max; X2/x2_max; X3/x3_max; X4/x4_max; X5/x5_max; X6/x6_max];

        % Running states through nerual network
        basis_func(:,j) = Basis_Func_84(xbar_k);
        lambda_k_plus_1 = Attitude_W' * Basis_Func_84(xbar_k);

        % Optimal control equation
        u_k_bar = (-Attitude_R^-1 * Attitude_G_bar(xbar_k).' * lambda_k_plus_1) *1000./Umax;

        % Discretized dynamics
        xbar_k_plus_1 = Attitude_F_bar(xbar_k) + Attitude_G_bar(xbar_k) * u_k_bar;

        % States through nerual network
        lambda_k_plus_2 = Attitude_W' * Basis_Func_84(xbar_k_plus_1);

        % Target costate equation
        A_k_plus_1 = A_non_dim(xbar_k_plus_1, dt, Ix, Iy, Iz, x1_max , x2_max, x3_max, x4_max, x5_max, x6_max);

        lambda_k_plus_1_target(:,j) = Attitude_Q * (xbar_k_plus_1) + discount*(A_k_plus_1.' * lambda_k_plus_2);
    end

    % Least squares to update network weights
    Attitude_W = (basis_func * basis_func')\(basis_func * lambda_k_plus_1_target');
    if isnan(Attitude_W)
        fprintf('Divergence in trainig \n')
        break
    end

    weight_plot(:,i) = reshape(Attitude_W.',1,[]);
    error(:, :) = lambda_k_plus_1 - lambda_k_plus_1_target;
    error_mae = mae(error(:,:));
    error_mse = mean(mean((error(:,:).^2) ));
    MAE_norm = error_mae/norm(Attitude_W);

    if mod(i, 100) == 0
        disp(['i = ', num2str(i)]);
        disp(['error_mae = ', num2str(error_mae)]);
        disp(['error_mse = ', num2str(error_mse)]);
        disp(['error_mae_NORM = ', num2str(MAE_norm)]);
    end
    if MAE_norm < threshold
        fprintf('converged\n')
        break
    end

end
fprintf("finished training")

%% test/train error
% [mae_train, mse_train, mae_test, mse_test] = att_test_train_error(x_k, Attitude_W, Attitude_R, Attitude_G, Attitude_F, Attitude_Q, a_col1, a_col2, a_col3, a_col4, a_col5, a_col6, PHI_min, PHI_max, THE_min, THE_max, PSI_min, PSI_max, p_min, p_max, q_min, q_max, r_min, r_max, N_patterns);
% fprintf("\ntraining error: mae = %f\n",mae_train)
% fprintf("training error: mse = %f\n",mse_train)
% fprintf("Test error: mae = %f\n", mae_test)
% fprintf("Test error: mse = %f\n",mse_test)

%% Simualtion

figure(1);
title('Attitude NN Weights', 'Interpreter', 'latex')
for k = 1:size(weight_plot,1) 
    hold on;
    plot(1:size(weight_plot,2) ,weight_plot(k,:)); 
end
xlabel('Iterations', 'Interpreter', 'latex');
ylabel('Weights', 'Interpreter', 'latex');
xlim([0 max_training_loop]);

% MAE_norm = error_mae/norm(Attitude_W);

Training_time = toc;
x1 = 1.3*[randn(3,1);0;0;0];
x2 = 1*[randn(3,1);0;0;0];
x3 = 1*[randn(3,1);0;0;0];


r = [0;0;0;0;0;0];


for i = 1:N
    time(i) = (i-1)*dt;
    r(:, i) = time(i)*0;
    % x1(:,i) = add_noise(x1(:,i),.05, 0);
    % x2(:,i) = add_noise(x2(:,i),.05, 2);
    % x3(:,i) = add_noise(x3(:,i),.05, 2);
    % x4(:,i) = add_noise(x4(:,i),.05, 2);
    u1(:,i) = -Attitude_R^-1 * Attitude_G(x1(:,i)-r(:, i))' * Attitude_W(:,:)' * Basis_Func_84(x1(:,i)-r(:, i));
    u2(:,i) = -Attitude_R^-1 * Attitude_G(x2(:,i)-r(:, i))' * Attitude_W(:,:)' * Basis_Func_84(x2(:,i)-r(:, i));
    u3(:,i) = -Attitude_R^-1 * Attitude_G(x3(:,i)-r(:, i))' * Attitude_W(:,:)' * Basis_Func_84(x3(:,i)-r(:, i));
    x1(:, i+1) = Attitude_F(x1(:,i)) + Attitude_G(x1(:,i)) * (u1(:,i).*Umax*1000);
    x2(:, i+1) = Attitude_F(x2(:,i)) + Attitude_G(x2(:,i)) * (u2(:,i).*Umax*1000);
    x3(:, i+1) = Attitude_F(x3(:,i)) + Attitude_G(x3(:,i)) * (u3(:,i).*Umax*1000);
end

figure(2)
subplot(3,1,1); plot(...
    time, r(1, 1:length(time)), 'r--',...
    time, x1(1, 1:length(time)),...
    time, x2(1, 1:length(time)),...
    time, x3(1, 1:length(time)))
title('Attitude Error', 'Interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex' )
ylabel('error in pitch', 'Interpreter', 'latex')
subplot(3,1,2); plot( ...
    time, r(2, 1:length(time)), 'r--',...
    time, x1(2, 1:length(time)), ...
    time, x2(2, 1:length(time)), ...
    time, x3(2, 1:length(time)))%, ...
xlabel('Time (sec)', 'Interpreter', 'latex' )
ylabel('error in roll', 'Interpreter', 'latex')
subplot(3,1,3); plot( ...
    time, r(3, 1:length(time)), 'r--',...
    time, x1(3, 1:length(time)), ...
    time, x2(3, 1:length(time)), ...
    time, x3(3, 1:length(time)))%, ...
xlabel('Time (sec)', 'Interpreter', 'latex' )
ylabel('error in yaw', 'Interpreter', 'latex')

figure(3)
subplot(3,1,1); plot(...
    time, r(4, 1:length(time)), 'r--',...
    time, x1(4, 1:length(time)),...
    time, x2(4, 1:length(time)),...
    time, x3(4, 1:length(time)))%,...
title('Error in Angular Velocities', 'Interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex' )
ylabel('$\dot{\phi}_b$', 'Interpreter', 'latex')
subplot(3,1,2); plot( ...
    time, r(5, 1:length(time)), 'r--',...
    time, x1(5, 1:length(time)), ...
    time, x2(5, 1:length(time)), ...
    time, x3(5, 1:length(time)))%, ...
xlabel('Time (sec)' , 'Interpreter', 'latex')
ylabel('$\dot{\theta}_b$', 'Interpreter', 'latex')
subplot(3,1,3); plot( ...
    time, r(6, 1:length(time)), 'r--',...
    time, x1(6, 1:length(time)), ...
    time, x2(6, 1:length(time)), ...
    time, x3(6, 1:length(time)))%, ...
xlabel('Time (sec)', 'Interpreter', 'latex' )
ylabel('$\dot{\psi}_b$', 'Interpreter', 'latex')


figure(4)
subplot(3,1,1); plot(...
    time, u1(1,1:length(time)),...
    time, u2(1,1:length(time)),...
    time, u3(1,1:length(time)))%,...
title('Controls', 'Interpreter', 'latex')
xlabel('Time (sec)' , 'Interpreter', 'latex')
ylabel('\tau_{x}')
subplot(3,1,2); plot(...
    time, u1(2,1:length(time)),...
    time, u2(2,1:length(time)),...
    time, u3(2,1:length(time)))%,...
xlabel('Time (sec)', 'Interpreter', 'latex' )
ylabel('\tau_{y}')
subplot(3,1,3); plot(...
    time, u1(3,1:length(time)),...
    time, u2(3,1:length(time)),...
    time, u3(3,1:length(time)))%,...
xlabel('Time (sec)' , 'Interpreter', 'latex')
ylabel('\tau_{z}')

figure(5)
grid on
hold on
title("Attitude States", 'Interpreter', 'latex')
plot3(x1(2,:), x1(2,:), x1(3,:), 'Linewidth', 1.5)
plot3(x2(1,:), x2(2,:), x2(3,:), 'Linewidth', 1.5)
plot3(x3(1,:), x3(2,:), x3(3,:), 'Linewidth', 1.5)
xlabel('\phi'), ylabel('\theta'), zlabel('\psi')
legend(["I.C 1", "I.C 2", "I.C 3"]);

fprintf('required time for training = %g sec\n', Training_time)

% save("test_workspace_att_V.mat",'Attitude_W','Attitude_R','Attitude_F','Attitude_G',"-v7.3")
% x_train_init = x_k_plus_1;
% target_init = lambda_k_plus_1_target;
% save("init_train_params.mat","x_train_init","target_init", "Attitude_Q","Attitude_R","N_patterns")

function noisy_vector = add_noise(state_vector, std_devs, noise_percent)
    noise = (noise_percent / 100) * std_devs .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end
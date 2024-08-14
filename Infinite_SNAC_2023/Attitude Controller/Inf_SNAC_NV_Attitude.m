clc; clear; close all;

addpath("/home/engelhardt/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions/")
% addpath('..\functions')

% Define plant dynamics
Ix = 10;   % moment of inertia (g*m^2) 
Iy = 10;  % moment of inertia (g*m^2)
Iz = 20;   % moment of inertia (g*m^2)
% non-dimesnionalize these dynamics

% Define domains of training
PHI_max = pi/6; PHI_min = -pi/6;
THE_max = pi/6; THE_min = -pi/6;
PSI_max = pi; PSI_min = -pi;

p_max =  pi/8; p_min =  -pi/8;
q_max =  pi/8; q_min =  -pi/8;
r_max =  pi/10; r_min =  -pi/10;

Attitude_f = @(x) [(x(4) + x(5)*(sin(x(1))*tan(x(2))) + x(6)*(cos(x(1))*tan(x(2))));
                   (x(5)*cos(x(1)) - x(6)*sin(x(1)));
                   (x(5)*sin(x(1))/cos(x(2)) + x(6)*cos(x(1))/cos(x(2)));
                   ((Iy - Iz) / Ix * x(5) * x(6));
                   ((Iz - Ix) / Iy * x(4) * x(6));
                   ((Ix - Iy) / Iz * x(4) * x(5))];
Attitude_g = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];

Attitude_f_bar = @(x_bar) [ (1/x1_bar)*(x_bar(4) + x_bar(5)*(sin(x_bar(1))*tan(x_bar(2))) + x_bar(6)*(cos(x_bar(1))*tan(x_bar(2))));
                            (1/x2_bar)*(x_bar(5)*cos(x_bar(1)) - x_bar(6)*sin(x_bar(1)));
                            (1/x3_bar)*(x_bar(5)*sin(x_bar(1))/cos(x_bar(2)) + x_bar(6)*cos(x_bar(1))/cos(x_bar(2)));
                            (1/x4_bar)*((Iy - Iz) / Ix * x_bar(5) * x_bar(6));
                            (1/x5_bar)*((Iz - Ix) / Iy * x_bar(4) * x_bar(6));
                            (1/x6_bar)*((Ix - Iy) / Iz * x_bar(4) * x_bar(5))      ];

Attitude_g_bar = [0 0 0; 0 0 0; 0 0 0; u1_max/Ix 0 0; 0 u2_max/Iy 0; 0 0 u3_max/Iz];

% Euler integration
Attitude_F = @(x) x + dt * Attitude_f(x);
Attitude_G = @(x) Attitude_g * dt;



% Define training Parameters
N_states =          6;
N_patterns =        1000;
max_training_loop = 5000;
threshold =         1e-5;
dt =                0.004;
% Attitude_Q = dt*diag([1e6,1e6,1e4,1e5,1e5,1e4]);
Attitude_Q = diag([1,1,1,1,1,1])*1000000;
Attitude_R = diag([1,1,1])*1;
% Attitude_R = dt*diag([0.5e4,0.5e4,0.5e4]);



% Partial x_k+1 / partial x_k
A = @(x)...
    [
    dt*(x(5)*cos(x(1))*tan(x(2)) - x(6)*sin(x(1))*tan(x(2))) + 1, dt*(x(6)*cos(x(1))*(tan(x(2))^2 + 1) + x(5)*sin(x(1))*(tan(x(2))^2 + 1)), 0, dt,  dt*sin(x(1))*tan(x(2)),  dt*cos(x(1))*tan(x(2)); 
    -dt*(x(6)*cos(x(1)) + x(5)*sin(x(1))), 1, 0,  0,  dt*cos(x(1)),  -dt*sin(x(1)); 
    dt*((x(5)*cos(x(1)))/cos(x(2)) - (x(6)*sin(x(1)))/cos(x(2))), dt*((x(6)*cos(x(1))*sin(x(2)))/cos(x(2))^2 + (x(5)*sin(x(1))*sin(x(2)))/cos(x(2))^2), 1,  0,  (dt*sin(x(1)))/cos(x(2)),  (dt*cos(x(1)))/cos(x(2)); 
    0, 0, 0,  1,  (dt*x(6)*(Iy - Iz))/Ix,  (dt*x(5)*(Iy - Iz))/Ix; 
     0, 0, 0,  -(dt*x(6)*(Ix - Iz))/Iy,  1,  -(dt*x(4)*(Ix - Iz))/Iy; 
    0, 0, 0, (dt*x(5)*(Ix - Iy))/Iz,  (dt*x(4)*(Ix - Iy))/Iz,  1; 
    ]; % row representation

% Define simulation parameters
t_f = 25;
discount = .99;

% Additonal variables
N_neurons = length(Basis_Func_84(ones(N_states,1)));
N = t_f/dt;   
Attitude_W = rand(N_neurons, N_states);
weight_plot = zeros((N_neurons* N_states),max_training_loop);

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
        x_k = [X1; X2; X3; X4; X5; X6];

        % Running states through nerual network
        basis_func(:,j) = Basis_Func_84(x_k);
        lambda_k_plus_1 = Attitude_W' * Basis_Func_84(x_k);

        % Optimal control equation
        u_k = -Attitude_R^-1 * Attitude_G(x_k).' * lambda_k_plus_1;

        % Discretized dynamics
        x_k_plus_1 = Attitude_F(x_k) + Attitude_G(x_k) * u_k;

        % States through nerual network
        lambda_k_plus_2 = Attitude_W' * Basis_Func_84(x_k_plus_1);

        % Target costate equation
        A_k_plus_1 = A(x_k_plus_1);
        lambda_k_plus_1_target(:,j) = Attitude_Q * (x_k_plus_1) + discount*(A_k_plus_1.' * lambda_k_plus_2);
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

    if mod(i, 100) == 0
        disp(['i = ', num2str(i)]);
        disp(['error_mae = ', num2str(error_mae)]);
        disp(['error_mse = ', num2str(error_mse)]);
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
for k = 1:size(weight_plot,1) 
    hold on;
    plot(1:size(weight_plot,2) ,weight_plot(k,:)); 
end
xlabel('Iterations');
ylabel('Weights');
xlim([0 i]);

MAE_norm = error_mae/norm(Attitude_W);

Training_time = toc;
x1 = 1.5*[randn(3,1);0;0;0];
x2 = [randn(3,1);0;0;0];
x3 = [randn(3,1);0;0;0];
x4 = [randn(3,1);0;0;0]; 

r = [0;0;0;0;0;0];


for i = 1:N
    time(i) = (i-1)*dt;
    r(:, i) = time(i)*0;
    x1(:,i) = add_noise(x1(:,i),.05, 5);
    % x2(:,i) = add_noise(x2(:,i),.05, 2);
    % x3(:,i) = add_noise(x3(:,i),.05, 2);
    % x4(:,i) = add_noise(x4(:,i),.05, 2);
    u1(:,i) = -Attitude_R^-1 * Attitude_G(x1(:,i)-r(:, i))' * Attitude_W(:,:)' * Basis_Func_84(x1(:,i)-r(:, i));
    u2(:,i) = -Attitude_R^-1 * Attitude_G(x2(:,i)-r(:, i))' * Attitude_W(:,:)' * Basis_Func_84(x2(:,i)-r(:, i));
    u3(:,i) = -Attitude_R^-1 * Attitude_G(x3(:,i)-r(:, i))' * Attitude_W(:,:)' * Basis_Func_84(x3(:,i)-r(:, i));
    u4(:,i) = -Attitude_R^-1 * Attitude_G(x4(:,i)-r(:, i))' * Attitude_W(:,:)' * Basis_Func_84(x4(:,i)-r(:, i));
    x1(:, i+1) = Attitude_F(x1(:,i)) + Attitude_G(x1(:,i)) * u1(:,i);
    x2(:, i+1) = Attitude_F(x2(:,i)) + Attitude_G(x2(:,i)) * u2(:,i);
    x3(:, i+1) = Attitude_F(x3(:,i)) + Attitude_G(x3(:,i)) * u3(:,i);
    x4(:, i+1) = Attitude_F(x4(:,i)) + Attitude_G(x4(:,i)) * u4(:,i);
end
figure(2)
    
subplot(3,1,1); plot(...
    time, r(1, 1:length(time)), 'r--',...
    time, x1(1, 1:length(time)),...
    time, x2(1, 1:length(time)),...
    time, x3(1, 1:length(time)),...
    time, x4(1, 1:length(time)))%,...
xlabel('Time (sec)' )
ylabel('error in pitch')
subplot(3,1,2); plot( ...
    time, r(2, 1:length(time)), 'r--',...
    time, x1(2, 1:length(time)), ...
    time, x2(2, 1:length(time)), ...
    time, x3(2, 1:length(time)), ...
    time, x4(2, 1:length(time)))%, ...
xlabel('Time (sec)' )
ylabel('error in roll')
subplot(3,1,3); plot( ...
    time, r(3, 1:length(time)), 'r--',...
    time, x1(3, 1:length(time)), ...
    time, x2(3, 1:length(time)), ...
    time, x3(3, 1:length(time)), ...
    time, x4(3, 1:length(time)))%, ...
xlabel('Time (sec)' )
ylabel('error in yaw')

figure(3)
subplot(3,1,1); plot(...
    time, r(4, 1:length(time)), 'r--',...
    time, x1(4, 1:length(time)),...
    time, x2(4, 1:length(time)),...
    time, x3(4, 1:length(time)),...
    time, x4(4, 1:length(time)))%,...
xlabel('Time (sec)' )
ylabel('error in pitch rate (rad/sec)')
subplot(3,1,2); plot( ...
    time, r(5, 1:length(time)), 'r--',...
    time, x1(5, 1:length(time)), ...
    time, x2(5, 1:length(time)), ...
    time, x3(5, 1:length(time)), ...
    time, x4(5, 1:length(time)))%, ...
xlabel('Time (sec)' )
ylabel('error in roll rate (rad/sec)')
subplot(3,1,3); plot( ...
    time, r(6, 1:length(time)), 'r--',...
    time, x1(6, 1:length(time)), ...
    time, x2(6, 1:length(time)), ...
    time, x3(6, 1:length(time)), ...
    time, x4(6, 1:length(time)))%, ...
xlabel('Time (sec)' )
ylabel('error in yaw rate (rad/sec)')

figure(4)
subplot(3,1,1); plot(...
    time, u1(1,1:length(time)),...
    time, u2(1,1:length(time)),...
    time, u3(1,1:length(time)),...
    time, u4(1,1:length(time)))%,...
xlabel('Time (sec)' )
ylabel('tau_x')
subplot(3,1,2); plot(...
    time, u1(2,1:length(time)),...
    time, u2(2,1:length(time)),...
    time, u3(2,1:length(time)),...
    time, u4(2,1:length(time)))%,...
xlabel('Time (sec)' )
ylabel('tau_y')
subplot(3,1,3); plot(...
    time, u1(3,1:length(time)),...
    time, u2(3,1:length(time)),...
    time, u3(3,1:length(time)),...
    time, u4(3,1:length(time)))%,...
xlabel('Time (sec)' )
ylabel('tau_z')


figure(5)
grid on
hold on
plot3(r(1,:), r(2,:), -r(3,:), '--', 'Linewidth', 1.5)
plot3(x1(1,:), x1(2,:), -x1(3,:), 'Linewidth', 1.5)
plot3(x2(1,:), x2(2,:), -x2(3,:), 'Linewidth', 1.5)
plot3(x3(1,:), x3(2,:), -x3(3,:), 'Linewidth', 1.5)
plot3(x4(1,:), x4(2,:), -x4(3,:), 'Linewidth', 1.5)
title('3D Trajectory')
xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
legend(["Reference trajectory", "SNAC"]);
fprintf('required time for training = %g sec\n', Training_time)

save("test_workspace_att_V.mat",'Attitude_W','Attitude_R','Attitude_F','Attitude_G',"-v7.3")
% save('test_workspace_att_V_10k_less_R.mat','Attitude_W','Attitude_R','Attitude_F','Attitude_G',"-v7.3")
x_train_init = x_k_plus_1;
target_init = lambda_k_plus_1_target;
save("init_train_params.mat","x_train_init","target_init", "Attitude_Q","Attitude_R","N_patterns")

function noisy_vector = add_noise(state_vector, std_devs, noise_percent)

    
    % Calculate the noise to be added
    noise = (noise_percent / 100) * std_devs .* randn(size(state_vector));
    
    % Add noise to the state vector
    noisy_vector = state_vector + noise;
end
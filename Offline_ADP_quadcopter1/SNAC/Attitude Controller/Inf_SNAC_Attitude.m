clc
clear
close all

addpath('..\functions')

% Define plant dynamics
m = 1;
Ix = 0.3;   % moment of inertia (kg*m^2)
Iy = 0.4;   % moment of inertia (kg*m^2)
Iz = 0.5;   % moment of inertia (kg*m^2)
Attitude_f = @(x) [(x(4) + x(5)*(sin(x(1))*tan(x(2))) + x(6)*(cos(x(1))*tan(x(2))));
                   (x(5)*cos(x(1)) - x(6)*sin(x(1)));
                   (x(5)*sin(x(1))/cos(x(2)) + x(6)*cos(x(1))/cos(x(2)));
                   ((Iy - Iz) / Ix * x(5) * x(6));
                   ((Iz - Ix) / Iy * x(4) * x(6));
                   ((Ix - Iy) / Iz * x(4) * x(5))];
Attitude_g = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];

% Define training Parameters
N_states =          6;
N_patterns =        500;
max_training_loop = 5000;
threshold =         1e-6;
dt =                0.004;
discount = 1;
Attitude_Q = diag([10,10,10,1,1,1])*1000;
Attitude_R = diag([1,1,1])*50;

% Define domains of training
PHI_max = pi/3; PHI_min = -pi/3;    % x1
THE_max = pi/3; THE_min = -pi/3;    % x2
PSI_max = pi/3; PSI_min = -pi/3;        % x3

p_max =  pi/3; p_min =  -pi/3;      % x4
q_max =  pi/3; q_min =  -pi/3;      % x5
r_max =  pi/3; r_min =  -pi/3;    % x6

% Euler integration
Attitude_F = @(x) x + dt * Attitude_f(x);
Attitude_G = @(x) Attitude_g * dt;

% Partial x_k+1 / partial x_k
A = @(x)...
    [
    dt*(x(5)*cos(x(1))*tan(x(2)) - x(6)*sin(x(1))*tan(x(2))) + 1, dt*(x(6)*cos(x(1))*(tan(x(2))^2 + 1) + x(5)*sin(x(1))*(tan(x(2))^2 + 1)), 0, dt,  dt*sin(x(1))*tan(x(2)),  dt*cos(x(1))*tan(x(2)); 
    -dt*(x(6)*cos(x(1)) + x(5)*sin(x(1))), 1, 0,  0,  dt*cos(x(1)),  -dt*sin(x(1)); 
    dt*((x(5)*cos(x(1)))/cos(x(2)) - (x(6)*sin(x(1)))/cos(x(2))), dt*((x(6)*cos(x(1))*sin(x(2)))/cos(x(2))^2 + (x(5)*sin(x(1))*sin(x(2)))/cos(x(2))^2), 1,  0,  (dt*sin(x(1)))/cos(x(2)),  (dt*cos(x(1)))/cos(x(2)); 
    0, 0, 0,  1,  (dt*x(6)*(Iy - Iz))/Ix,  (dt*x(5)*(Iy - Iz))/Ix; 
    0, 0, 0,  -(dt*x(6)*(Ix - Iz))/Iy,  1,  -(dt*x(4)*(Ix - Iz))/Iy; 
    0, 0, 0,  (dt*x(5)*(Ix - Iy))/Iz,  (dt*x(4)*(Ix - Iy))/Iz,  1; 
    ]; % row representation

% Define simulation parameters
t_f = 6;
N_neurons = length(Basis_Func_84(ones(N_states,1)))
N = t_f/dt;   
Attitude_W = randn(N_neurons, N_states);
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
                %% add random noise to the x_k_plus_1

        % States through nerual network
        lambda_k_plus_2 = Attitude_W' * Basis_Func_84(x_k_plus_1);

        % Target costate equation
        A_k_plus_1 = A(x_k_plus_1);
        lambda_k_plus_1_target(:,j) = Attitude_Q * (x_k_plus_1) ...
            + discount*A_k_plus_1.' * lambda_k_plus_2;
    end

    % Least squares to update network weights
    Attitude_W = (basis_func * basis_func')\(basis_func * lambda_k_plus_1_target');
    if isnan(Attitude_W)
        fprintf('Divergence in trainig \n')
        break
    end

    % Check for convergence
    error(:, :) = Attitude_W' * basis_func - lambda_k_plus_1_target;
    % if mae(error(:,:))< threshold
    %     fprintf('Weights converged, mae = %f \n', mae(error(:,:)))
    %     break
    % end
    weight_plot(:,i) = reshape(Attitude_W.',1,[]);
    error(:, :) = lambda_k_plus_1 - lambda_k_plus_1_target;
    error_mae = mae(error(:,:));
    error_mse = mean(mean((error(:,:).^2) ));
    MAE_norm = error_mae/norm(Attitude_W);

    if mod(i, 100) == 0
        disp(['i = ', num2str(i)]);
        disp(['error_mae = ', num2str(error_mae)]);
        % disp(['error_mse = ', num2str(error_mse)]);
        disp(['error_mae_NORM = ', num2str(MAE_norm)]);
    end
end
 toc

 %% test/train error
% [mae_train, mse_train, mae_test, mse_test] = att_test_train_error(x_k, Attitude_W, Attitude_R, Attitude_G, Attitude_F, Attitude_Q, a_col1, a_col2, a_col3, a_col4, a_col5, a_col6, PHI_min, PHI_max, THE_min, THE_max, PSI_min, PSI_max, p_min, p_max, q_min, q_max, r_min, r_max, N_patterns);
% fprintf("\ntraining error: mae = %f\n",mae_train)
% fprintf("training error: mse = %f\n",mse_train)
% fprintf("Test error: mae = %f\n", mae_test)
% fprintf("Test error: mse = %f\n",mse_test)

%% Simualtion
save("SNAC_att_dim.mat",'Attitude_W','Attitude_R',"Attitude_Q",'Attitude_F','Attitude_G')

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
    u1(:,i) = -Attitude_R^-1 * Attitude_G( (x1(:,i)-r(:, i)) )' * Attitude_W(:,:)' * Basis_Func_84( (x1(:,i)-r(:, i)) );
    u2(:,i) = -Attitude_R^-1 * Attitude_G( (x2(:,i)-r(:, i)) )' * Attitude_W(:,:)' * Basis_Func_84( (x2(:,i)-r(:, i)) );
    u3(:,i) = -Attitude_R^-1 * Attitude_G( (x3(:,i)-r(:, i)) )' * Attitude_W(:,:)' * Basis_Func_84( (x3(:,i)-r(:, i)) );
    x1(:, i+1) = Attitude_F(x1(:,i)) + Attitude_G(x1(:,i)) * (u1(:,i));
    x2(:, i+1) = Attitude_F(x2(:,i)) + Attitude_G(x2(:,i)) * (u2(:,i));
    x3(:, i+1) = Attitude_F(x3(:,i)) + Attitude_G(x3(:,i)) * (u3(:,i));
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

function noisy_vector = add_noise(state_vector, std_devs, noise_percent)
    noise = (noise_percent / 100) * std_devs .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end

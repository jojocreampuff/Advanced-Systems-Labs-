clc; clear;

% Define training Parameters
N_states = 6;
N_patterns = 1000;
max_training_loop = 1000;
threshold = 1e-10;
dt = 0.01;
Position_Q = dt*10*diag([1,1,1,1,1,1])/dt;
Position_R = dt*diag([10,10,10])/dt; 

% Define domains of training
X_max = 10; X_min = -10;
Y_max = 10; Y_min = -10;
Z_max = 10; Z_min = -10;

u_max = 10; u_min = -10;
v_max = 10; v_min = -10;
w_max = 10; w_min = -10;

% Define plant dynamics
grav = 9.81;
Position_f = @(x) [x(4); x(5); x(6); 0; 0; grav];
Position_g = @(x) [0 0 0; 0 0 0; 0 0 0; -1 0 0; 0 -1 0; 0 0 -1];

% Euler integration
Position_F = @(x) x + dt * Position_f(x);
Position_G = @(x) Position_g(x) * dt;

% Partial x_k+1 / partial x_k grad(f(x) + g(x) *u)
A = @(x)...
    [
    1, 0, 0, dt,  0,  0; 
    0, 1, 0,  0, dt,  0; 
    0, 0, 1,  0,  0, dt; 
    0, 0, 0,  1,  0,  0; 
    0, 0, 0,  0,  1,  0; 
    0, 0, 0,  0,  0,  1; 
    ]; % row representation

% Define simulation parameters
t_f = 50;
Position_F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t
            (sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000
            (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5
            -.1];
time = 0:dt:t_f-dt;
x1 = [0;0;0;0;0;0]; 
r = [0;0;0;0;0;0];
for i = 1:length(time)
    r_position(:,i) = Position_F_r(time(i));
end

r = r_position;

% Additonal variables
N = t_f/dt;   
N_neurons = length(Basis_Func_pos(ones(N_states,1)));
Position_W = randn(N_neurons, N_states);
error_mae = zeros(1,max_training_loop);
error_mse = zeros(1,max_training_loop);
MAE_norm = zeros(1, max_training_loop);
tic

% Nonvectorzied SNAC training loop
for i = 1:max_training_loop
    basis_func = zeros(N_neurons, N_patterns); 
    lambda_k_plus_1_target = zeros(N_states, N_patterns); 
    % Generating target costate for all number of patterns
    for j = 1 : N_patterns
        X1 = X_min + (X_max - X_min) * rand;%(1, N_patterns);
        X2 = Y_min + (Y_max - Y_min) * rand;%(1, N_patterns);
        X3 = Z_min + (Z_max - Z_min) * rand;%(1, N_patterns);    
        X4 = u_min + (u_max - u_min) * rand;%(1, N_patterns);
        X5 = v_min + (v_max - v_min) * rand;%(1, N_patterns);
        X6 = w_min + (w_max - w_min) * rand;%(1, N_patterns);
        
        % Random error states within defined domain of trainig
        x_k = [X1; X2; X3; X4; X5; X6];

        % Running states through nerual network
        basis_func(:,j) = Basis_Func_pos(x_k);
        lambda_k_plus_1 = Position_W' * Basis_Func_pos(x_k); %step 1

        % Optimal control equation
        u_k = (-Position_R^-1 * Position_G(x_k).' * lambda_k_plus_1); %step 2

        % Discretized dynamics
        x_k_plus_1 = Position_F(x_k) + Position_G(x_k) * u_k; % step 3
        %% add wind noise to the x_k_plus_1
        % x_k_plus_1(4:6) = x_k_plus_1(4:6) + dist_vector(1:3,j);

        % States through nerual network
        lambda_k_plus_2 = Position_W' * Basis_Func_pos(x_k_plus_1); % step 4

        % Target costate equation
        A_k_plus_1 = A(x_k_plus_1);
        lambda_k_plus_1_target(:,j) = Position_Q * (x_k_plus_1) + A_k_plus_1.' * lambda_k_plus_2;
    end

    % Least squares to update network weights
    Position_W = (basis_func * basis_func')\(basis_func * lambda_k_plus_1_target');
    weight_plot(:,i) = reshape(Position_W.',1,[]);
    % Check for convergence
    error(:, :) = Position_W' * basis_func - lambda_k_plus_1_target;
    error_mae(i) = mae(error(:,:));
    error_mse(i) = mean(mean((error(:,:).^2) ));

    % if error_mae(i) < threshold
    %     fprintf('converged\n')
    %     break
    % end

    if mod(i, 100) == 0
        disp(['i = ', num2str(i)]);
        disp(['error_mae = ', num2str(error_mae(i))]);
        % disp(['error_mse = ', num2str(error_mse(i))]);
    end
end
Training_time = toc;
fprintf('required time for training = %g sec\n', Training_time)

%% Test/Train error
% [mae_train, mse_train, mae_test, mse_test] = pos_test_train_error(x_k, Position_W, Position_R, Position_G, Position_F, Position_Q, A, N_states, N_patterns, X_min, X_max, Y_min, Y_max, Z_min, Z_max, u_min, u_max, v_min, v_max, w_min, w_max);
% fprintf("\ntraining error: mae = %f\n",mae_train)
% fprintf("training error: mse = %f\n",mse_train)
% fprintf("Test error: mae = %f\n", mae_test)
% fprintf("Test error: mse = %f\n",mse_test)

figure;
% title("Position NN weights", 'Interpreter', 'latex')
for k = 1:size(weight_plot,1) 
    hold on;
    plot(1:size(weight_plot,2) ,weight_plot(k,:)); 
end
xlabel('Iterations', 'Interpreter', 'latex');
ylabel('$\widehat{W}_{p}$', 'Interpreter','latex')
set(gca, 'FontSize', 16, ...
         'TickLabelInterpreter','latex', ...
         'XGrid','on', 'YGrid','on', ...
         'GridLineStyle','--')   % dashed grid
grid on
xlim([0 i]); 

% save("z_Offline_SNAC_pos_data_MORE_R.mat",'Position_W',"Position_Q",'Position_R','Position_g','Position_G',"dt")
% save("SNAC_pos_WIND.mat",'Position_W',"Position_Q",'Position_R','Position_F','Position_G',"dt","grav")

% save("C:\Users\randy\Box\Randy Research\SNAC_Paper\codes for paper\harsher_cost_comparisons\SNAC_pos_harsh.mat",'Position_W',"Position_Q",'Position_R','Position_F','Position_G',"dt","grav"')

%% Simualtion
x1 = [0;0;0;0;0;0]; 
x2 = [2*randn(2,1);0;0;0;0];
x3 = [2*randn(2,1);0;0;0;0];
x4 = [2*randn(2,1);0;0;0;0]; 
instant_cost = zeros(1, N); % Instantaneous cost
cumulative_cost = zeros(1, N); % Integrated cost

for i = 1:N
    u1(:,i) = -Position_R^-1 * Position_G(x1(:,i)-r(:, i))' * Position_W' * Basis_Func_pos(x1(:,i)-r(:, i));
    u2(:,i) = -Position_R^-1 * Position_G(x2(:,i)-r(:, i))' * Position_W' * Basis_Func_pos(x2(:,i)-r(:, i));
    u3(:,i) = -Position_R^-1 * Position_G(x3(:,i)-r(:, i))' * Position_W' * Basis_Func_pos(x3(:,i)-r(:, i));
    u4(:,i) = -Position_R^-1 * Position_G(x4(:,i)-r(:, i))' * Position_W' * Basis_Func_pos(x4(:,i)-r(:, i));
    x1(:, i+1) = Position_F(x1(:,i)) + Position_G(x1(:,i)) * (u1(:,i));
    x2(:, i+1) = Position_F(x2(:,i)) + Position_G(x2(:,i)) * (u2(:,i));
    x3(:, i+1) = Position_F(x3(:,i)) + Position_G(x3(:,i)) * (u3(:,i));
    x4(:, i+1) = Position_F(x4(:,i)) + Position_G(x4(:,i)) * (u4(:,i));
    instant_cost(i) = x1(:,i)' * Position_Q * x1(:,i) + u1(:,i)' * Position_R * u1(:,i);
   if i > 1
        cumulative_cost(i) = cumulative_cost(i-1) + instant_cost(i) * dt;
   end
end

% save("training_plot_pos.mat")

% figure
% grid on
% hold on
% plot3(r(1,:), r(2,:), -r(3,:), '--', 'Linewidth', 1.5)
% plot3(x1(1,:), x1(2,:), -x1(3,:), 'Linewidth', 1.5)
% plot3(x2(1,:), x2(2,:), -x2(3,:), 'Linewidth', 1.5)
% plot3(x3(1,:), x3(2,:), -x3(3,:), 'Linewidth', 1.5)
% plot3(x4(1,:), x4(2,:), -x4(3,:), 'Linewidth', 1.5)
% title('3D Trajectory')
% xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
% legend(["Reference trajectory", "SNAC"]);
% 
% figure;
% subplot(3,1,1); plot(...
%     time, r(1, 1:length(time)), 'r--',...
%     time, x1(1, 1:length(time)),...
%     time, x2(1, 1:length(time)),...
%     time, x3(1, 1:length(time)),...
%     time, x4(1, 1:length(time)))
% 
% xlabel('Time (sec)' )
% ylabel('x')
% subplot(3,1,2); plot( ...
%     time, r(2, 1:length(time)), 'r--',...
%     time, x1(2, 1:length(time)), ...
%     time, x2(2, 1:length(time)), ...
%     time, x3(2, 1:length(time)), ...
%     time, x4(2, 1:length(time)))
% xlabel('Time (sec)' )
% ylabel('y')
% subplot(3,1,3); plot( ...
%     time, r(3, 1:length(time)), 'r--',...
%     time, x1(3, 1:length(time)), ...
%     time, x2(3, 1:length(time)), ...
%     time, x3(3, 1:length(time)), ...
%     time, x4(3, 1:length(time)))
% xlabel('Time (sec)' )
% ylabel('z')
% 
% figure;
% subplot(3,1,1); plot(...
%     time, r(4, 1:length(time)), 'r--',...
%     time, x1(4, 1:length(time)),...
%     time, x2(4, 1:length(time)),...
%     time, x3(4, 1:length(time)),...
%     time, x4(4, 1:length(time)))
% xlabel('Time (sec)' )
% ylabel('x_dot')
% subplot(3,1,2); plot( ...
%     time, r(5, 1:length(time)), 'r--',...
%     time, x1(5, 1:length(time)), ...
%     time, x2(5, 1:length(time)), ...
%     time, x3(5, 1:length(time)), ...
%     time, x4(5, 1:length(time)))
% xlabel('Time (sec)' )
% ylabel('y_dot')
% subplot(3,1,3); plot( ...
%     time, r(6, 1:length(time)), 'r--',...
%     time, x1(6, 1:length(time)), ...
%     time, x2(6, 1:length(time)), ...
%     time, x3(6, 1:length(time)), ...
%     time, x4(6, 1:length(time)))
% xlabel('Time (sec)' )
% ylabel('z_dot')
% 
% figure;
% subplot(3,1,1); plot(...
%     time, u1(1,1:length(time)),...
%     time, u2(1,1:length(time)),...
%     time, u3(1,1:length(time)),...
%     time, u4(1,1:length(time)))
% xlabel('Time (sec)' )
% ylabel('u1')
% subplot(3,1,2); plot(...
%     time, u1(2,1:length(time)),...
%     time, u2(2,1:length(time)),...
%     time, u3(2,1:length(time)),...
%     time, u4(2,1:length(time)))
% xlabel('Time (sec)' )
% ylabel('u2')
% subplot(3,1,3); plot(...
%     time, u1(3,1:length(time)),...
%     time, u2(3,1:length(time)),...
%     time, u3(3,1:length(time)),...
%     time, u4(3,1:length(time)))
% xlabel('Time (sec)' )
% ylabel('u3')
% 
% % Plot Instantaneous Cost
% figure;
% plot(time, instant_cost, 'b', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Instantaneous Cost');
% title('Instantaneous Cost Over Time');
% grid on;
% 
% % Plot Cumulative Cost
% figure;
% plot(time, cumulative_cost, 'r', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Cumulative Cost');
% title('Total Accumulated Cost Over Time');
% grid on;


function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end

function noisy_vector = add_noise(state_vector, std_devs, noise_percent)
    noise = (noise_percent / 100) * std_devs .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end

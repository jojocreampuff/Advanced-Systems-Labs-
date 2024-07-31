clc; clear; close all;

addpath("/home/engelhardt/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions/")
% addpath('..\functions')

% Define plant dynamics
grav = 9.81;
Position_f = @(x) [x(4); x(5); x(6); 0; 0; grav];
Position_g = @(x) [0 0 0; 0 0 0; 0 0 0; -1 0 0; 0 -1 0; 0 0 -1];


% Define training Parameters
N_states = 6;
N_patterns = 1000;
max_training_loop = 40000;
threshold = 1e-5;
dt = 0.004;
Position_Q = diag([100000,100000,100000,100000,100000,100000]);
Position_R = diag([10000,10000,10000]); % SITL has too much control input
% Position_R = dt*diag([0.5e4,0.5e4,0.5e4]);
% Position_Q = diag([1000,1000,1000,1000,1000,1000]);
% Position_R = diag([100,100,100]);

% Define domains of training
X_max = 1000; X_min = -1000;
Y_max = 1000; Y_min = -1000;
Z_max = 100; Z_min = -100;

u_max = 20; u_min = -20;
v_max = 20; v_min = -20;
w_max = 20; w_min = -20;

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
t_f = 100;
Position_F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -1*t
            (sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000
            (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5
            -1];

% Euler integration
Position_F = @(x) x + dt * Position_f(x);
Position_G = @(x) Position_g(x) * dt;

% Additonal variables
N = t_f/dt;   
N_neurons = length(Basis_Func_pos(ones(N_states,1)));
Position_W = randn(N_neurons, N_states)*.1;

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
        
        % Random states within defined domain of trainig
        x_k = [X1; X2; X3; X4; X5; X6];

        % Running states through nerual network
        basis_func(:,j) = Basis_Func_pos(x_k);
        lambda_k_plus_1 = Position_W' * Basis_Func_pos(x_k); %step 1

        % Optimal control equation
        u_k = -Position_R^-1 * Position_G(x_k).' * lambda_k_plus_1; %step 2

        % Discretized dynamics
        x_k_plus_1 = Position_F(x_k) + Position_G(x_k) * u_k; % step 3

        % States through nerual network
        lambda_k_plus_2 = Position_W' * Basis_Func_pos(x_k_plus_1); % step 4

        % Target costate equation
        A_k_plus_1 = A(x_k_plus_1);
        lambda_k_plus_1_target(:,j) = Position_Q * (x_k_plus_1) + A_k_plus_1.' * lambda_k_plus_2;
    end

    % Least squares to update network weights
    Position_W = (basis_func * basis_func')\(basis_func * lambda_k_plus_1_target');
    if isnan(Position_W)
        fprintf('Divergence in trainig \n')
        break
    end
    weight_plot(:,i) = reshape(Position_W.',1,[]);
    % Check for convergence
    error(:, :) = Position_W' * basis_func - lambda_k_plus_1_target;
    if mae(error(:,:))< threshold
        fprintf('converged\n')
        break
    end
    error_test = mae(error(:,:));

    if mod(i, 100) == 0
        fprintf('At: %g iterations, the MAE is %f \n', i,error_test)
    end
end

figure(1);
for k = 1:size(weight_plot,1) 
    hold on;
    plot(1:size(weight_plot,2) ,weight_plot(k,:)); 
end
xlabel('Iterations');
ylabel('Weights');
xlim([0 i]); 

Training_time = toc;
fprintf('required time for training = %g sec\n', Training_time)

%% Simualtion

x1 = [0;0;0;0;0;0]; % 9 different inital conditions??
x1 = rand(6,1)*5;
x2 = [-1;-1;0;0;0;0];
x3 = [1;-1;0;0;0;0];
x4 = [-1;1;0;0;0;0]; 
x5 = [1;1;0;0;0;0];
x6 = [5; 5;0;0;0;0]; 
x7 = [-5; 5;0;0;0;0]; 
x8 = [5;-5;0;0;0;0];
x9 = [-5;-5;0;0;0;0]; 

r = [0;0;0;0;0;0];

time = 0:dt:t_f-dt;
for i = 1:length(time)
    r_position(:,i) = Position_F_r(time(i));
end

smooth_r_position = smooth(r_position(1:3,:), x1(1:3)', x1(4:6)', time);

r = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];

for i = 1:N
    u1(:,i) = -Position_R^-1 * Position_G(x1(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x1(:,i)-r(:, i));
    u2(:,i) = -Position_R^-1 * Position_G(x2(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x2(:,i)-r(:, i));
    u3(:,i) = -Position_R^-1 * Position_G(x3(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x3(:,i)-r(:, i));
    u4(:,i) = -Position_R^-1 * Position_G(x4(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x4(:,i)-r(:, i));
    u5(:,i) = -Position_R^-1 * Position_G(x5(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x5(:,i)-r(:, i));
    u6(:,i) = -Position_R^-1 * Position_G(x6(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x6(:,i)-r(:, i));
    u7(:,i) = -Position_R^-1 * Position_G(x7(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x7(:,i)-r(:, i));
    u8(:,i) = -Position_R^-1 * Position_G(x8(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x8(:,i)-r(:, i));
    u9(:,i) = -Position_R^-1 * Position_G(x9(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x9(:,i)-r(:, i));
    x1(:, i+1) = Position_F(x1(:,i)) + Position_G(x1(:,i)) * u1(:,i);
    x2(:, i+1) = Position_F(x2(:,i)) + Position_G(x2(:,i)) * u2(:,i);
    x3(:, i+1) = Position_F(x3(:,i)) + Position_G(x3(:,i)) * u3(:,i);
    x4(:, i+1) = Position_F(x4(:,i)) + Position_G(x4(:,i)) * u4(:,i);
    x5(:, i+1) = Position_F(x5(:,i)) + Position_G(x5(:,i)) * u5(:,i);
    x6(:, i+1) = Position_F(x6(:,i)) + Position_G(x6(:,i)) * u6(:,i);
    x7(:, i+1) = Position_F(x7(:,i)) + Position_G(x7(:,i)) * u7(:,i);
    x8(:, i+1) = Position_F(x8(:,i)) + Position_G(x8(:,i)) * u8(:,i);
    x9(:, i+1) = Position_F(x9(:,i)) + Position_G(x9(:,i)) * u9(:,i);
end
figure
    
subplot(3,1,1); plot(...
    time, r(1, 1:length(time)), 'r--',...
    time, x1(1, 1:length(time)),...
    time, x2(1, 1:length(time)),...
    time, x3(1, 1:length(time)),...
    time, x4(1, 1:length(time)),...
    time, x5(1, 1:length(time)),...
    time, x6(1, 1:length(time)),...
    time, x7(1, 1:length(time)),...
    time, x8(1, 1:length(time)),...
    time, x9(1, 1:length(time)))
xlabel('Time (sec)' )
ylabel('x')
subplot(3,1,2); plot( ...
    time, r(2, 1:length(time)), 'r--',...
    time, x1(2, 1:length(time)), ...
    time, x2(2, 1:length(time)), ...
    time, x3(2, 1:length(time)), ...
    time, x4(2, 1:length(time)), ...
    time, x5(2, 1:length(time)),...
    time, x6(2, 1:length(time)),...
    time, x7(2, 1:length(time)),...
    time, x8(2, 1:length(time)),...
    time, x9(2, 1:length(time)))
xlabel('Time (sec)' )
ylabel('y')
subplot(3,1,3); plot( ...
    time, r(3, 1:length(time)), 'r--',...
    time, x1(3, 1:length(time)), ...
    time, x2(3, 1:length(time)), ...
    time, x3(3, 1:length(time)), ...
    time, x4(3, 1:length(time)), ...
    time, x5(3, 1:length(time)),...
    time, x6(3, 1:length(time)),...
    time, x7(3, 1:length(time)),...
    time, x8(3, 1:length(time)),...
    time, x9(3, 1:length(time)))
xlabel('Time (sec)' )
ylabel('y')

figure
subplot(3,1,1); plot(...
    time, r(4, 1:length(time)), 'r--',...
    time, x1(4, 1:length(time)),...
    time, x2(4, 1:length(time)),...
    time, x3(4, 1:length(time)),...
    time, x4(4, 1:length(time)),...
    time, x5(4, 1:length(time)),...
    time, x6(4, 1:length(time)),...
    time, x7(4, 1:length(time)),...
    time, x8(4, 1:length(time)),...
    time, x9(4, 1:length(time)))
xlabel('Time (sec)' )
ylabel('x')
subplot(3,1,2); plot( ...
    time, r(5, 1:length(time)), 'r--',...
    time, x1(5, 1:length(time)), ...
    time, x2(5, 1:length(time)), ...
    time, x3(5, 1:length(time)), ...
    time, x4(5, 1:length(time)), ...
    time, x5(5, 1:length(time)),...
    time, x6(5, 1:length(time)),...
    time, x7(5, 1:length(time)),...
    time, x8(5, 1:length(time)),...
    time, x9(5, 1:length(time)))
xlabel('Time (sec)' )
ylabel('y')
subplot(3,1,3); plot( ...
    time, r(6, 1:length(time)), 'r--',...
    time, x1(6, 1:length(time)), ...
    time, x2(6, 1:length(time)), ...
    time, x3(6, 1:length(time)), ...
    time, x4(6, 1:length(time)), ...
    time, x5(6, 1:length(time)),...
    time, x6(6, 1:length(time)),...
    time, x7(6, 1:length(time)),...
    time, x8(6, 1:length(time)),...
    time, x9(6, 1:length(time)))
xlabel('Time (sec)' )
ylabel('y')

figure
subplot(3,1,1); plot(...
    time, u1(1,1:length(time)),...
    time, u2(1,1:length(time)),...
    time, u3(1,1:length(time)),...
    time, u4(1,1:length(time)),...
    time, u5(1,1:length(time)),...
    time, u6(1,1:length(time)),...
    time, u7(1,1:length(time)),...
    time, u8(1,1:length(time)),...
    time, u9(1,1:length(time)))
xlabel('Time (sec)' )
ylabel('u1')
subplot(3,1,2); plot(...
    time, u1(2,1:length(time)),...
    time, u2(2,1:length(time)),...
    time, u3(2,1:length(time)),...
    time, u4(2,1:length(time)),...
    time, u5(2,1:length(time)),...
    time, u6(2,1:length(time)),...
    time, u7(2,1:length(time)),...
    time, u8(2,1:length(time)),...
    time, u9(2,1:length(time)))
xlabel('Time (sec)' )
ylabel('u2')
subplot(3,1,3); plot(...
    time, u1(3,1:length(time)),...
    time, u2(3,1:length(time)),...
    time, u3(3,1:length(time)),...
    time, u4(3,1:length(time)),...
    time, u5(3,1:length(time)),...
    time, u6(3,1:length(time)),...
    time, u7(3,1:length(time)),...
    time, u8(3,1:length(time)),...
    time, u9(3,1:length(time)))
xlabel('Time (sec)' )
ylabel('u3')

figure
grid on
hold on
plot3(r(1,:), r(2,:), -r(3,:), '--', 'Linewidth', 1.5)
plot3(x1(1,:), x1(2,:), -x1(3,:), 'Linewidth', 1.5)
plot3(x2(1,:), x2(2,:), -x2(3,:), 'Linewidth', 1.5)
plot3(x3(1,:), x3(2,:), -x3(3,:), 'Linewidth', 1.5)
plot3(x4(1,:), x4(2,:), -x4(3,:), 'Linewidth', 1.5)
plot3(x5(1,:), x5(2,:), -x5(3,:), 'Linewidth', 1.5)
plot3(x6(1,:), x6(2,:), -x6(3,:), 'Linewidth', 1.5)
plot3(x7(1,:), x7(2,:), -x7(3,:), 'Linewidth', 1.5)
plot3(x8(1,:), x8(2,:), -x8(3,:), 'Linewidth', 1.5)
plot3(x9(1,:), x9(2,:), -x9(3,:), 'Linewidth', 1.5)
title('3D Trajectory')
xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
legend(["Reference trajectory", "SNAC"]);

save("test_workspace_pos.mat","-v7.3")

function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end


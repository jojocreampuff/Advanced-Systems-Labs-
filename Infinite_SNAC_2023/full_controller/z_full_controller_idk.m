clc; clear; close all;
% my pc path
% addpath("/home/engelhardt/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions/")

% lab work stations paths
% addpath("/home/users10/re606359/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions")
% addpath("C:\Users\re606359\Desktop\Advanced-Systems-Labs-\Infinite_SNAC_2023\functions " )

% Define plant dynamics
Ix = 0.01;   % moment of inertia (kg*m^2)
Iy = 0.01;   % moment of inertia (kg*m^2)
Iz = 0.02;   % moment of inertia (kg*m^2)
m = .77; grav = 9.81;

N_states = 12;
N_patterns = 1000;
max_training_loop = 2000;
threshold = 1e-5;
dt = 0.004; t_f = 50; N = t_f/dt;
N_neurons = length(Basis_Func_full(ones(N_states,1)));
W_full = rand(N_neurons, N_states)*.01;
weight_plot = zeros((N_neurons* N_states),max_training_loop);
 
discount = 0.9;
Q = diag([1,1,1,1,1,1,1,1,1,1,1,1])*10;
R = diag([1,1,1,1])*10;

% training domain for all 12 states
X_max = 100; X_min = -100;
Y_max = 100; Y_min = -100;
Z_max = 100; Z_min = -100;

u_max = 20; u_min = -20;
v_max = 20; v_min = -20;
w_max = 20; w_min = -20;

PHI_max = pi/6; PHI_min = -pi/6;
THE_max = pi/6; THE_min = -pi/6;
PSI_max = pi; PSI_min = -pi;

p_max =  pi/8; p_min =  -pi/8;
q_max =  pi/8; q_min =  -pi/8;
r_max =  pi/10; r_min =  -pi/10;

% define max state values and controls to non-dim
x1_max = X_max;
x2_max = Y_max;
x3_max = Z_max;
x4_max = u_max;
x5_max = v_max;
x6_max = w_max;
x7_max = PHI_max;
x8_max = THE_max;
x9_max = PSI_max;
x10_max = p_max;
x11_max = q_max;
x12_max = r_max;
max_states = [x1_max; x2_max; x3_max; x4_max; x5_max; x6_max; x7_max; x8_max; x9_max; x10_max; x11_max; x12_max]*1.2;

Ft_max = 40; tx_max = 2; ty_max = 2; tz_max = 1;
max_controls = [Ft_max; tx_max; ty_max; tz_max];

% Euler integration
F_bar = @(x_bar,grav,Ix,Iy,Iz, max_states) x_bar + dt * Full_f(x_bar,grav,Ix,Iy,Iz, max_states);
G_bar = @(x_bar,m,Ix,Iy,Iz,max_states, max_controls) Full_g(x_bar,m,Ix,Iy,Iz,max_states, max_controls) * dt;

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
        X7 = PHI_min + (PHI_max - PHI_min) * rand;%(1, N_patterns);
        X8 = THE_min + (THE_max - THE_min) * rand;%(1, N_patterns);
        X9 = PSI_min + (PSI_max - PSI_min) * rand;%(1, N_patterns);    
        X10 = p_min + (p_max - p_min) * rand;%(1, N_patterns);
        X11 = q_min + (q_max - q_min) * rand;%(1, N_patterns);
        X12 = r_min + (r_max - r_min) * rand;%(1, N_patterns);

        xbar_k = [X1; X2; X3; X4; X5; X6; X7; X8; X9; X10; X11; X12] ./max_states;

        % Running states through nerual network
        basis_func(:,j) = Basis_Func_full(xbar_k);
        lambda_k_plus_1 = W_full' * basis_func(:,j);

        % Optimal control equation
        u_k_bar = (-R^-1 * G_bar(xbar_k,m,Ix,Iy,Iz,max_states, max_controls).' * lambda_k_plus_1) ./max_controls;
        
        if u_k_bar(1) < 0
            u_k_bar(1) = 0;
        end

        % Discretized non-dim dynamics
        xbar_k_plus_1 = F_bar(xbar_k, grav,Ix,Iy,Iz, max_states) + G_bar(xbar_k, m,Ix,Iy,Iz,max_states, max_controls) * u_k_bar;

        % States through nerual network
        lambda_k_plus_2 = W_full' * Basis_Func_full(xbar_k_plus_1);

        % Target costate equation
        Abar_k_plus_1 = A_full_non_dim(xbar_k_plus_1, dt, m, Ix, Iy, Iz, max_states, max_controls(1), u_k_bar(1));

        lambda_k_plus_1_target(:,j) = Q * (xbar_k_plus_1) + discount*(Abar_k_plus_1.' * lambda_k_plus_2);
    end

    % Least squares to update network weights
    W_full = (basis_func * basis_func')\(basis_func * lambda_k_plus_1_target');
    
    if isnan(W_full)
        fprintf('Divergence in trainig \n')
        break
    end

    weight_plot(:,i) = reshape(W_full.',1,[]);
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
Training_time = toc;
fprintf('required time for training = %g sec\n', Training_time)

%% Simualtion

figure(1);
for k = 1:size(weight_plot,1) 
    hold on;
    plot(1:size(weight_plot,2) ,weight_plot(k,:)); 
end
xlabel('Iterations');
ylabel('Weights');
xlim([0 i]);

MAE_norm = error_mae/norm(W_full);

% I.C
%       x,y             z       u       v       w      phi     the     psi      p   q   r
x1 = [5*randn(2,1);     0;      0;      0;      0;      0;      0;      0;      0;  0;  0];
x2 = [5*randn(2,1);     0;      0;      0;      0;      pi/2;   0;      0;      0;  0;  0];
x3 = [5*randn(2,1);     0;      0;      0;      0;      pi/4;   0;      0;      0;  0;  0];
x4 = [5*randn(2,1);     0;      0;      0;      0;      pi;     0;      0;      0;  0;  0];

% load path for previous solution
load("path_full_dynamics", "pos_states","ref_angles")
r = [pos_states; zeros(6,1),ref_angles];

% regular dynamcis 
Full_F = @(x,grav,Ix,Iy,Iz) x + dt * Full_f_225(x,grav,Ix,Iy,Iz); % discretized drift dynamics
Full_G = @(x,m,Ix,Iy,Iz) dt * Full_g_225(x,m,Ix,Iy,Iz);   

for i = 1:N
    % x1(:,i) = add_noise(x1(:,i),.05, 0);
    % x2(:,i) = add_noise(x2(:,i),.05, 2);
    % x3(:,i) = add_noise(x3(:,i),.05, 2);
    % x4(:,i) = add_noise(x4(:,i),.05, 2);
    u1(:,i) = -R^-1 * Full_G(x1(:,i)-r(:, i),m,Ix,Iy,Iz)' * W_full(:,:)' * Basis_Func_full(x1(:,i)-r(:, i));
    u2(:,i) = -R^-1 * Full_G(x2(:,i)-r(:, i),m,Ix,Iy,Iz)' * W_full(:,:)' * Basis_Func_full(x2(:,i)-r(:, i));
    u3(:,i) = -R^-1 * Full_G(x3(:,i)-r(:, i),m,Ix,Iy,Iz)' * W_full(:,:)' * Basis_Func_full(x3(:,i)-r(:, i));
    u4(:,i) = -R^-1 * Full_G(x4(:,i)-r(:, i),m,Ix,Iy,Iz)' * W_full(:,:)' * Basis_Func_full(x4(:,i)-r(:, i));
    x1(:, i+1) = Full_F(x1(:,i),grav,Ix,Iy,Iz) + Full_G(x1(:,i),m,Ix,Iy,Iz) * (u1(:,i).*max_controls);
    x2(:, i+1) = Full_F(x2(:,i),grav,Ix,Iy,Iz) + Full_G(x2(:,i),m,Ix,Iy,Iz) * (u2(:,i).*max_controls);
    x3(:, i+1) = Full_F(x3(:,i),grav,Ix,Iy,Iz) + Full_G(x3(:,i),m,Ix,Iy,Iz) * (u3(:,i).*max_controls);
    x4(:, i+1) = Full_F(x4(:,i),grav,Ix,Iy,Iz) + Full_G(x4(:,i),m,Ix,Iy,Iz) * (u4(:,i).*max_controls);
end

figure(2)
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
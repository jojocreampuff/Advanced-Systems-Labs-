clc; clear; close all;

addpath("/home/engelhardt/Desktop/Advanced-Systems-Labs-/Infinite_SNAC_2023/functions/")


% Define plant dynamics
grav = 9.81;
Position_f = @(x) [x(4); x(5); x(6); 0; 0; grav];
Position_g = @(x) [0 0 0; 0 0 0; 0 0 0; -1 0 0; 0 -1 0; 0 0 -1];

% Define training Parameters
N_states = 6;
N_patterns = 10000;
max_training_loop = 5000;
threshold = 1e-7;
dt = 0.004;
Position_Q = diag([100000,100000,100000,100000,100000,100000]);
Position_R = diag([1,1,1])*10000; % SITL has too much control input

% Define domains of training
X_max = 1000; X_min = -1000;
Y_max = 1000; Y_min = -1000;
Z_max = 100; Z_min = -100;

u_max = 20; u_min = -20;
v_max = 20; v_min = -20;
w_max = 20; w_min = -20;

A_k_plus_1_T = @(x)...
    [
    ones(1,length(x(1,:))); zeros(1,length(x(1,:))); zeros(1,length(x(1,:))); zeros(1,length(x(1,:)));  zeros(1,length(x(1,:)));  zeros(1,length(x(1,:))); 
    zeros(1,length(x(1,:))); ones(1,length(x(1,:))); zeros(1,length(x(1,:)));  zeros(1,length(x(1,:))); zeros(1,length(x(1,:)));  zeros(1,length(x(1,:))); 
    zeros(1,length(x(1,:))); zeros(1,length(x(1,:))); ones(1,length(x(1,:)));  zeros(1,length(x(1,:)));  zeros(1,length(x(1,:))); zeros(1,length(x(1,:))); 
    dt*ones(1,length(x(1,:))); zeros(1,length(x(1,:))); zeros(1,length(x(1,:)));  ones(1,length(x(1,:)));  zeros(1,length(x(1,:)));  zeros(1,length(x(1,:))); 
    zeros(1,length(x(1,:))); dt*ones(1,length(x(1,:))); zeros(1,length(x(1,:)));  zeros(1,length(x(1,:)));  ones(1,length(x(1,:)));  zeros(1,length(x(1,:))); 
    zeros(1,length(x(1,:))); zeros(1,length(x(1,:))); dt*ones(1,length(x(1,:)));  zeros(1,length(x(1,:)));  zeros(1,length(x(1,:)));  ones(1,length(x(1,:))); 
    ];

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
Position_W = rand(N_neurons, N_states)*.1;

tic
% Nonvectorzied SNAC training loop
for i = 1:max_training_loop

        X1 = X_min + (X_max - X_min) * rand(1, N_patterns);
        X2 = Y_min + (Y_max - Y_min) * rand(1, N_patterns);
        X3 = Z_min + (Z_max - Z_min) * rand(1, N_patterns);    
        X4 = u_min + (u_max - u_min) * rand(1, N_patterns);
        X5 = v_min + (v_max - v_min) * rand(1, N_patterns);
        X6 = w_min + (w_max - w_min) * rand(1, N_patterns);
        
        % Random states within defined domain of trainig
        x_k = [X1; X2; X3; X4; X5; X6];

        % Running states through nerual network
        LHS = Basis_Func_pos(x_k);
        lambda_k_plus_1 = Position_W' * LHS;

        % Optimal control equation
        u_k = -Position_R^-1 * Position_G(x_k)' * lambda_k_plus_1;

        % Discretized dynamics
        x_k_plus_1 = Position_F(x_k) + Position_G(x_k)* u_k;

        % States through nerual network
        lambda_k_plus_2 = Position_W' * Basis_Func_pos(x_k_plus_1);

        % Target costate equation
        A = A_k_plus_1_T(x_k_plus_1); %A_k_plus_1'

        L = lambda_k_plus_2; % for simplicity

%         KEEP_DIMS =     ...
%             [
%     A(1,:).*L(1,:) + A(2,:).*L(2,:) + A(3,:).*L(3,:) + A(4,:).*L(4,:) + A(5,:).*L(5,:) + A(6,:).*L(6,:); 
%     A(7,:).*L(1,:) + A(8,:).*L(2,:) + A(9,:).*L(3,:) + A(10,:).*L(4,:) + A(11,:).*L(5,:) + A(12,:).*L(6,:);
%     A(13,:).*L(1,:) + A(14,:).*L(2,:) + A(15,:).*L(3,:) + A(16,:).*L(4,:) + A(17,:).*L(5,:) + A(18,:).*L(6,:);
%     A(19,:).*L(1,:) + A(20,:).*L(2,:) + A(21,:).*L(3,:) + A(22,:).*L(4,:) + A(23,:).*L(5,:) + A(24,:).*L(6,:);
%     A(25,:).*L(1,:) + A(26,:).*L(2,:) + A(27,:).*L(3,:) + A(28,:).*L(4,:) + A(29,:).*L(5,:) + A(30,:).*L(6,:);
%     A(31,:).*L(1,:) + A(32,:).*L(2,:) + A(33,:).*L(3,:) + A(34,:).*L(4,:) + A(35,:).*L(5,:) + A(36,:).*L(6,:);
%     ];
        KEEP_DIMS =     ...
            [
    A(1,:).*L(1,:) ; 
    A(8,:).*L(2,:) ;
    A(15,:).*L(3,:);
    A(19,:).*L(1,:) + A(22,:).*L(4,:) ;
    A(26,:).*L(2,:) + A(29,:).*L(5,:) ;
    A(33,:).*L(3,:) + A(36,:).*L(6,:) ;
    ];
        lambda_k_plus_1_target = Position_Q * (x_k_plus_1) + .99*KEEP_DIMS;


    % Least squares to update network weights
    Position_W = (LHS * LHS')\(LHS * lambda_k_plus_1_target');

    weight_plot(:,i) = reshape(Position_W.',1,[]);
    % Check for convergence
    error(:, :) = Position_W' * LHS - lambda_k_plus_1_target;
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

x1 = [0;0;0;0;0;0]; 
r = [0;0;0;0;0;0];

time = 0:dt:t_f-dt;
for i = 1:length(time)
    r_position(:,i) = Position_F_r(time(i));
end

smooth_r_position = smooth(r_position(1:3,:), x1(1:3)', x1(4:6)', time);

r = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];

for i = 1:N
    u1(:,i) = -Position_R^-1 * Position_G(x1(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x1(:,i)-r(:, i));

    x1(:, i+1) = Position_F(x1(:,i)) + Position_G(x1(:,i)) * u1(:,i);
end

stdv = std(x1(:,end-1)-r, 0 , 2);
stdv(3) = (stdv(1)+stdv(2))/2;
x1(:,1) = [0;0;0;0;0;0]; 

for i = 1:N
    x1(:,i) = add_noise(x1(:,i),stdv, .5);
    u1(:,i) = -Position_R^-1 * Position_G(x1(:,i)-r(:, i))' * Position_W(:,:)' * Basis_Func_pos(x1(:,i)-r(:, i));

    x1(:, i+1) = Position_F(x1(:,i)) + Position_G(x1(:,i)) * u1(:,i);
end

figure(2)
grid on
hold on
plot3(r(1,:), r(2,:), -r(3,:), '--', 'Linewidth', 1.5)
plot3(x1(1,:), x1(2,:), -x1(3,:), 'Linewidth', 1.5)

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

function noisy_vector = add_noise(state_vector, std_devs, noise_percent)

    
    % Calculate the noise to be added
    noise = (noise_percent / 100) * std_devs .* randn(size(state_vector));
    
    % Add noise to the state vector
    noisy_vector = state_vector + noise;
end




% A_k_plus_1_T = @(x)...
%     [
%     ones(1,length(x(1))), zeros(1,length(x(1))), zeros(1,length(x(1))), zeros(1,length(x(1))),  zeros(1,length(x(1))),  zeros(1,length(x(1))); 
%     zeros(1,length(x(1))), ones(1,length(x(1))), zeros(1,length(x(1))),  zeros(1,length(x(1))), zeros(1,length(x(1))),  zeros(1,length(x(1))); 
%     zeros(1,length(x(1))), zeros(1,length(x(1))), ones(1,length(x(1))),  zeros(1,length(x(1))),  zeros(1,length(x(1))), zeros(1,length(x(1))); 
%     dt*ones(1,length(x(1))), zeros(1,length(x(1))), zeros(1,length(x(1))),  ones(1,length(x(1))),  zeros(1,length(x(1))),  zeros(1,length(x(1))); 
%     zeros(1,length(x(1))), dt*ones(1,length(x(1))), zeros(1,length(x(1))),  zeros(1,length(x(1))),  ones(1,length(x(1))),  zeros(1,length(x(1))); 
%     zeros(1,length(x(1))), zeros(1,length(x(1))), dt*ones(1,length(x(1))),  zeros(1,length(x(1))),  zeros(1,length(x(1))),  ones(1,length(x(1))); 
%     ]; % This is vectorized A transposed already to simplify things


%         KEEP_DIMS =     ...
%             [
%     A(1,1).*L(1,:) + A(1,2).*L(2,:) + A(1,3).*L(3,:) + A(1,4).*L(4,:) + A(1,5).*L(5,:) + A(1,6).*L(6,:); 
%     A(2,1).*L(1,:) + A(2,2).*L(2,:) + A(2,3).*L(3,:) + A(2,4).*L(4,:) + A(2,5).*L(5,:) + A(2,6).*L(6,:); 
%     A(3,1).*L(1,:) + A(3,2).*L(2,:) + A(3,3).*L(3,:) + A(3,4).*L(4,:) + A(3,5).*L(5,:) + A(3,6).*L(6,:); 
%     A(4,1).*L(1,:) + A(4,2).*L(2,:) + A(4,3).*L(3,:) + A(4,4).*L(4,:) + A(4,5).*L(5,:) + A(4,6).*L(6,:); 
%     A(5,1).*L(1,:) + A(5,2).*L(2,:) + A(5,3).*L(3,:) + A(5,4).*L(4,:) + A(5,5).*L(5,:) + A(5,6).*L(6,:); 
%     A(6,1).*L(1,:) + A(6,2).*L(2,:) + A(6,3).*L(3,:) + A(6,4).*L(4,:) + A(6,5).*L(5,:) + A(6,6).*L(6,:); 
%     ];
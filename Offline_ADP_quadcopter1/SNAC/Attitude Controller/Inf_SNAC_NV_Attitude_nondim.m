clc; clear;
% NOTE: This code is set up for non_wind training
% note changed cost function
Ix = 0.3;
Iy = 0.4;
Iz = 0.5;
m = 1; grav = 9.81;
% non-dimesnionalize these dynamics

PHI_max = pi/4; PHI_min = -pi/4;    % x1
THE_max = pi/4; THE_min = -pi/4;    % x2
PSI_max = pi/4; PSI_min = -pi/4;        % x3

p_max =  pi/4; p_min =  -pi/4;      % x4
q_max =  pi/4; q_min =  -pi/4;      % x5
r_max =  pi/4; r_min =  -pi/4;    % x6

% PHI_max = pi/3.5; PHI_min = -pi/3.5;    % x1
% THE_max = pi/3.5; THE_min = -pi/3.5;    % x2
% PSI_max = pi/3.5; PSI_min = -pi/3.5;        % x3
% 
% p_max =  pi/3.5; p_min =  -pi/3.5;      % x4
% q_max =  pi/3.5; q_min =  -pi/3.5;      % x5
% r_max =  pi/3.5; r_min =  -pi/3.5;    % x6

% PHI_max = pi/2.5; PHI_min = -pi/2.5;    % x1
% THE_max = pi/2.5; THE_min = -pi/2.5;    % x2
% PSI_max = pi/2.5; PSI_min = -pi/2.5;        % x3
% 
% p_max =  pi/2.5; p_min =  -pi/2.5;      % x4
% q_max =  pi/2.5; q_min =  -pi/2.5;      % x5
% r_max =  pi/2.5; r_min =  -pi/2.5;    % x6

x1_max = PHI_max;
x2_max = THE_max;
x3_max = PSI_max;
x4_max = p_max;
x5_max = q_max;
x6_max = r_max;
max_states = [x1_max; x2_max; x3_max; x4_max; x5_max; x6_max];
ft_max = 2*m*grav; % N
motor_max = ft_max/4; % 5 N per motor
drone_radius = .2; % lever Arm

u1_max = motor_max*2*drone_radius; 
u2_max = u1_max; 
u3_max = u1_max;
Umax = [u1_max; u2_max; u3_max]; % unit is Nm

% Define training Parameters
N_states =          6;
N_patterns =        1000;
max_training_loop = 1000;
threshold =         1e-5;
dt =                0.01;
discount = 1;
Attitude_Q = [300,300,100,10,10,10].*diag([1,1,1,1,1,1]);
Attitude_R = dt*diag([1/dt,1/dt,1/dt]);

% Define simulation parameters
t_f = 10;
N_neurons = length(a_Offline_SNAC_phi_att(ones(N_states,1)));
N = t_f/dt;   
Attitude_W = rand(N_neurons, N_states);
weight_plot = zeros((N_neurons* N_states),max_training_loop);

Attitude_f = @(x) [(x(4) + x(5)*(sin(x(1))*tan(x(2))) + x(6)*(cos(x(1))*tan(x(2))));
                   (x(5)*cos(x(1)) - x(6)*sin(x(1)));
                   (x(5)*sin(x(1))/cos(x(2)) + x(6)*cos(x(1))/cos(x(2)));
                   ((Iy - Iz) / Ix * x(5) * x(6));
                   ((Iz - Ix) / Iy * x(4) * x(6));
                   ((Ix - Iy) / Iz * x(4) * x(5))];
Attitude_g = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];

Attitude_f_bar = @(x_bar) [ (1/x1_max)*(x_bar(4)*x4_max + x_bar(5)*x4_max*(sin( x_bar(1)*x1_max )*tan( x_bar(2)*x2_max )) + x_bar(6)*x6_max*(cos( x_bar(1)*x1_max )*tan( x_bar(2)*x2_max )));
                            (1/x2_max)*(x_bar(5)*x5_max*cos( x_bar(1)*x1_max ) - x_bar(6)*x6_max*sin( x_bar(1)*x1_max ));
                            (1/x3_max)*(x_bar(5)*x5_max*sin( x_bar(1)*x1_max )/cos( x_bar(2)*x2_max ) + x_bar(6)*x6_max*cos( x_bar(1)*x1_max )/cos( x_bar(2)*x2_max ));
                            (1/x4_max)*((Iy - Iz) / Ix * x_bar(5) * x_bar(6)* x5_max* x6_max); 
                            (1/x5_max)*((Iz - Ix) / Iy * x_bar(4) * x_bar(6)* x4_max* x6_max);
                            (1/x6_max)*((Ix - Iy) / Iz * x_bar(4) * x_bar(5)* x4_max* x5_max)];

Attitude_g_bar = [0 0 0; 0 0 0; 0 0 0; u1_max/(Ix*x4_max) 0 0; 0 u2_max/(Iy*x5_max) 0; 0 0 u3_max/(Iz*x6_max)];

% Euler integration
Attitude_F_bar = @(x_bar) x_bar + dt * Attitude_f_bar(x_bar);
Attitude_G_bar = @(x_bar) Attitude_g_bar * dt;

Attitude_F = @(x) x + dt * Attitude_f(x);
Attitude_G = @(x) Attitude_g * dt;
% load("wind_simulation.mat")
error_mae = zeros(1,max_training_loop);
error_mse = zeros(1,max_training_loop);
MAE_norm = zeros(1, max_training_loop);
MSE_norm = zeros(1, max_training_loop);

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
        basis_func(:,j) = a_Offline_SNAC_phi_att(xbar_k);
        lambda_k_plus_1 = Attitude_W' * a_Offline_SNAC_phi_att(xbar_k);

        % Optimal control equation
        % u_k_bar = (-Attitude_R^-1 * Attitude_G_bar(xbar_k).' * lambda_k_plus_1) * 1000; % this is only if MOI is in grams-m^3
        u_k_bar = (-Attitude_R^-1 * Attitude_G_bar(xbar_k).' * lambda_k_plus_1); %./Umax;

        % Discretized dynamics
        xbar_k_plus_1 = Attitude_F_bar(xbar_k) + Attitude_G_bar(xbar_k) * u_k_bar;
        %% add random noise to the x_k_plus_1
        % odds = rand;
        % x_k_plus_1 = xbar_k_plus_1.*max_states; % denorm
        % x_k_plus_1(4:6) = x_k_plus_1(4:6) + dist_vector(4:6,j); % add the disturbance torques from wind simulation
        % xbar_k_plus_1 = x_k_plus_1./max_states;

        % States through nerual network
        lambda_k_plus_2 = Attitude_W' * a_Offline_SNAC_phi_att(xbar_k_plus_1);

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
    error_mae(i) = mae(error(:,:));
    error_mse(i) = mean(mean((error(:,:).^2) ));
    MSE_norm(i) = error_mse(i)/norm(Attitude_W);
    MAE_norm(i) = error_mae(i)/norm(Attitude_W);

    if mod(i, 100) == 0
        disp(['i = ', num2str(i)]);
        % disp(['error_mae = ', num2str(error_mae)]);
        % disp(['error_mse = ', num2str(error_mse)]);
        disp(['error_mae_NORM = ', num2str(MAE_norm(i))]);
    end

end
Training_time = toc;
fprintf("finished training")
% save("z_offline_SNAC_att_NEW.mat",'Attitude_W','Attitude_R',"Attitude_Q",'Attitude_F','Attitude_G',"Attitude_G_bar","Attitude_F_bar","max_states")
% save("C:\Users\randy\Box\Randy Research\SNAC_Paper\codes for paper\harsher_cost_comparisons\SNAC_att_harsh.mat", ...
%      'Attitude_W','Attitude_R','Attitude_Q','Attitude_F','Attitude_G', ...
%      'Attitude_G_bar','Attitude_F_bar','max_states','-v7.3')
%% Simualtion

figure;
for k = 1:size(weight_plot,1) 
    hold on;
    plot(1:size(weight_plot,2) ,weight_plot(k,:)); 
end
xlabel('Iterations', 'Interpreter', 'latex');
ylabel('$\widehat{W}_{a}$', 'Interpreter','latex')
set(gca, 'FontSize', 16, ...
         'TickLabelInterpreter','latex', ...
         'XGrid','on', 'YGrid','on', ...
         'GridLineStyle','--')   % dashed grid
grid on
xlim([0 max_training_loop]);

% MAE_norm = error_mae/norm(Attitude_W);


x1 = [randn(3,1);0;0;0];
x2 = 1*[randn(3,1);0;0;0];
x3 = 1*[randn(3,1);0;0;0];

instant_cost = zeros(1, N); % Instantaneous cost
cumulative_cost = zeros(1, N); % Integrated cost


r = [0;0;0;0;0;0];


for i = 1:N
    time(i) = (i-1)*dt;
    r(:, i) = time(i)*0;
    % how do the unit work here? DO I need to use Attutude_G_bar?
    u1(:,i) = -Attitude_R^-1 * Attitude_G_bar( (x1(:,i)-r(:, i))./max_states )' * Attitude_W(:,:)' * a_Offline_SNAC_phi_att( (x1(:,i)-r(:, i))./max_states );
    u2(:,i) = -Attitude_R^-1 * Attitude_G_bar( (x2(:,i)-r(:, i))./max_states )' * Attitude_W(:,:)' * a_Offline_SNAC_phi_att( (x2(:,i)-r(:, i))./max_states );
    u3(:,i) = -Attitude_R^-1 * Attitude_G_bar( (x3(:,i)-r(:, i))./max_states )' * Attitude_W(:,:)' * a_Offline_SNAC_phi_att( (x3(:,i)-r(:, i))./max_states );
    x1(:, i+1) = Attitude_F(x1(:,i)) + Attitude_G(x1(:,i)) * (u1(:,i).*Umax);%*1000);
    x2(:, i+1) = Attitude_F(x2(:,i)) + Attitude_G(x2(:,i)) * (u2(:,i).*Umax);%*1000);
    x3(:, i+1) = Attitude_F(x3(:,i)) + Attitude_G(x3(:,i)) * (u3(:,i).*Umax);%*1000);
    % find cost of tracking during horizon
    instant_cost(i) = x1(:,i)' * Attitude_Q * x1(:,i) + u1(:,i)' * Attitude_R * u1(:,i);
   if i > 1
        cumulative_cost(i) = cumulative_cost(i-1) + instant_cost(i) * dt;
   end

end

% save("training_plots_att.mat")
% 
% figure(2)
% subplot(3,1,1); plot(...
%     time, r(1, 1:length(time)), 'r--',...
%     time, x1(1, 1:length(time)),...
%     time, x2(1, 1:length(time)),...
%     time, x3(1, 1:length(time)))
% title('Attitude Error', 'Interpreter', 'latex')
% xlabel('Time (sec)', 'Interpreter', 'latex' )
% ylabel('error in pitch', 'Interpreter', 'latex')
% subplot(3,1,2); plot( ...
%     time, r(2, 1:length(time)), 'r--',...
%     time, x1(2, 1:length(time)), ...
%     time, x2(2, 1:length(time)), ...
%     time, x3(2, 1:length(time)))%, ...
% xlabel('Time (sec)', 'Interpreter', 'latex' )
% ylabel('error in roll', 'Interpreter', 'latex')
% subplot(3,1,3); plot( ...
%     time, r(3, 1:length(time)), 'r--',...
%     time, x1(3, 1:length(time)), ...
%     time, x2(3, 1:length(time)), ...
%     time, x3(3, 1:length(time)))%, ...
% xlabel('Time (sec)', 'Interpreter', 'latex' )
% ylabel('error in yaw', 'Interpreter', 'latex')
% 
% figure(3)
% subplot(3,1,1); plot(...
%     time, r(4, 1:length(time)), 'r--',...
%     time, x1(4, 1:length(time)),...
%     time, x2(4, 1:length(time)),...
%     time, x3(4, 1:length(time)))%,...
% title('Error in Angular Velocities', 'Interpreter', 'latex')
% xlabel('Time (sec)', 'Interpreter', 'latex' )
% ylabel('$\dot{\phi}_b$', 'Interpreter', 'latex')
% subplot(3,1,2); plot( ...
%     time, r(5, 1:length(time)), 'r--',...
%     time, x1(5, 1:length(time)), ...
%     time, x2(5, 1:length(time)), ...
%     time, x3(5, 1:length(time)))%, ...
% xlabel('Time (sec)' , 'Interpreter', 'latex')
% ylabel('$\dot{\theta}_b$', 'Interpreter', 'latex')
% subplot(3,1,3); plot( ...
%     time, r(6, 1:length(time)), 'r--',...
%     time, x1(6, 1:length(time)), ...
%     time, x2(6, 1:length(time)), ...
%     time, x3(6, 1:length(time)))%, ...
% xlabel('Time (sec)', 'Interpreter', 'latex' )
% ylabel('$\dot{\psi}_b$', 'Interpreter', 'latex')
% 
% 
% figure(4)
% subplot(3,1,1); plot(...
%     time, u1(1,1:length(time)),...
%     time, u2(1,1:length(time)),...
%     time, u3(1,1:length(time)))%,...
% title('Controls', 'Interpreter', 'latex')
% xlabel('Time (sec)' , 'Interpreter', 'latex')
% ylabel('\tau_{x}')
% subplot(3,1,2); plot(...
%     time, u1(2,1:length(time)),...
%     time, u2(2,1:length(time)),...
%     time, u3(2,1:length(time)))%,...
% xlabel('Time (sec)', 'Interpreter', 'latex' )
% ylabel('\tau_{y}')
% subplot(3,1,3); plot(...
%     time, u1(3,1:length(time)),...
%     time, u2(3,1:length(time)),...
%     time, u3(3,1:length(time)))%,...
% xlabel('Time (sec)' , 'Interpreter', 'latex')
% ylabel('\tau_{z}')
% 
% figure(5)
% grid on
% hold on
% title("Attitude States", 'Interpreter', 'latex')
% plot3(x1(2,:), x1(2,:), x1(3,:), 'Linewidth', 1.5)
% plot3(x2(1,:), x2(2,:), x2(3,:), 'Linewidth', 1.5)
% plot3(x3(1,:), x3(2,:), x3(3,:), 'Linewidth', 1.5)
% xlabel('\phi'), ylabel('\theta'), zlabel('\psi')
% legend(["I.C 1", "I.C 2", "I.C 3"]);

% Plot Instantaneous Cost
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

fprintf('required time for training = %g sec\n', Training_time)
% save("training_plots_att.mat")
% save("SNAC_att_NEW.mat",'Attitude_W','Attitude_R',"Attitude_Q",'Attitude_F','Attitude_G',"Attitude_G_bar","Attitude_F_bar","max_states","-v7.3")
% save("SNAC_att_WIND.mat",'Attitude_W','Attitude_R',"Attitude_Q",'Attitude_F','Attitude_G',"Attitude_G_bar","Attitude_F_bar","max_states","-v7.3")

function noisy_vector = add_noise(state_vector, std_devs, noise_percent)
    noise = (noise_percent / 100) * std_devs .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end
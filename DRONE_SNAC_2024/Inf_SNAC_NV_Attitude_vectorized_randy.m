clc
clear
close all
% Define plant dynamics
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
N_states = 6;
N_patterns = 1000;
max_training_loop = 3000;
threshold = 1e-5;
dt = 0.001;
discount = 0.96;
Attitude_Q = dt*diag([4e7,4e7,4e7,3e5,3e5,3e5]);
Attitude_R = dt*diag([1e3,1e3,1e3]);
% Attitude_Q = dt*diag([3e6,3e6,3e4,1e8,1e8,1e7]);
% Attitude_R = dt*diag([2e3,2e3,2e3]);

% Define domains of training
PHI_max = 0.5; PHI_min = -0.5; % +- 1 rad (57.3 deg)
THE_max = 0.5; THE_min = -0.5;
PSI_max = 0.5; PSI_min = -0.5;

p_max =  0.5; p_min =  -0.5;
q_max =  0.5; q_min =  -0.5;
r_max =  0.5; r_min =  -0.5;

% PHI_max = 1; PHI_min = -1; % +- 1 rad (57.3 deg)
% THE_max = 1; THE_min = -1;
% PSI_max = 1; PSI_min = -1;
% 
% p_max =  1; p_min =  -1;
% q_max =  1; q_min =  -1;
% r_max =  1; r_min =  -1;

% PHI_max = 1.3; PHI_min = -1.3; % +- 1 rad (57.3 deg)
% THE_max = 1.3; THE_min = -1.3;
% PSI_max = 1.3; PSI_min = -1.3;
% 
% p_max =  1.3; p_min =  -1.3;
% q_max =  1.3; q_min =  -1.3;
% r_max =  1.3; r_min =  -1.3;

% Partial x_k+1 / partial x_k


%% 
% I took the columns of A defined above and vectorized them, The "columns"
% now 6 x # training samples can be sliced easier then f I called each
% element of a 1x36 stacked matrix

a_col1 = @(x)...
    [
    dt*(x(5,:).*cos(x(1,:)).*tan(x(2,:)) - x(6,:).*sin(x(1,:)).*tan(x(2,:))) + ones(1,length(x(1,:)));
    -dt* (x(6,:).*cos(x(1,:)) + (x(5,:).*sin(x(1,:))));
    dt *( (x(5,:).*cos(x(1,:)))./cos(x(2,:)) - (x(6,:).*sin(x(1,:)))./cos(x(2,:)));
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)))    ]; % this is column 1 of A, each A_col1(i,:) can be element wise with L(i,:) for i = 1:6

a_col2 = @(x)...
    [
    dt*( x(6,:).*cos(x(1,:)).*((tan(x(2,:)).^2) + ones(1,length(x(1,:)))) + x(5,:).*sin(x(1,:)).*((tan(x(2,:)).^2) + ones(1,length(x(1,:)))));
    ones(1,length(x(1,:)));
    dt*((x(6,:).*cos(x(1,:)).*sin(x(2,:))) ./ (cos(x(2,:)).^2) + (x(5,:).*sin(x(1,:)).*sin(x(2,:)))./(cos(x(2,:)).^2));
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)))
    ];

a_col3 = @(x)...
    [
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)));
    ones(1,length(x(1,:)))
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)));
    ];

a_col4 = @(x)...
    [
    dt*ones(1,length(x(1,:)));
    zeros(1,length(x(1,:)));
    zeros(1,length(x(1,:)));
    ones(1,length(x(1,:)));
    -(dt*x(6,:)*(Ix - Iz)) / Iy;
    (dt*x(5,:)*(Ix - Iy)) / Iz
    ];

a_col5 = @(x)...
    [
    dt*sin(x(1,:)).*tan(x(2,:));
    dt*cos(x(1,:));
    (dt*sin(x(1,:)))./cos(x(2,:));
    (dt*x(6,:)*(Iy - Iz))/Ix;
    ones(1,length(x(1,:)));
    (dt*x(4,:)*(Ix - Iy))/Iz       
    ];

a_col6 = @(x)...
    [
    dt*cos(x(1,:)).*tan(x(2,:)); 
    -dt*sin(x(1,:)); 
    (dt*cos(x(1,:)))./cos(x(2,:)); 
    (dt*x(5,:)*(Iy - Iz))/Ix; 
    -(dt*x(4,:)*(Ix - Iz))/Iy; 
    ones(1,length(x(1,:))); 
    ]; 

% Define simulation parameters
t_f = 10;

% Euler integration
Attitude_F = @(x) x + dt * Attitude_f(x);
Attitude_G = @(x) Attitude_g * dt;

% Additonal variables
N_neurons = length(basis_2(ones(N_states,1)));
N = t_f/dt;   
Attitude_W = rand(N_neurons, N_states)*.1;
weight_plot = zeros((N_neurons* N_states),max_training_loop);

tic
fprintf("starting training\n")

for i = 1:max_training_loop

    X1 = PHI_min + (PHI_max - PHI_min) * rand(1, N_patterns);
    X2 = THE_min + (THE_max - THE_min) * rand(1, N_patterns);
    X3 = PSI_min + (PSI_max - PSI_min) * rand(1, N_patterns);    
    X4 = p_min + (p_max - p_min) * rand(1, N_patterns);
    X5 = q_min + (q_max - q_min) * rand(1, N_patterns);
    X6 = r_min + (r_max - r_min) * rand(1, N_patterns); 

    x_k = [X1; X2; X3; X4; X5; X6];

    LHS = basis_2(x_k);
    lambda_k_plus_1 = Attitude_W' * LHS;

    % Optimal control equation
    u_k = -Attitude_R^-1 * Attitude_G(x_k)' * lambda_k_plus_1;

    % Discretized dynamics
    x_k_plus_1 = Attitude_F(x_k) + Attitude_G(x_k)* u_k;

    % States through nerual network
    lambda_k_plus_2 = Attitude_W' * basis_2(x_k_plus_1);

    % Target costate equation
    A_col1 = a_col1(x_k_plus_1);
    A_col2 = a_col2(x_k_plus_1);
    A_col3 = a_col3(x_k_plus_1);
    A_col4 = a_col4(x_k_plus_1);
    A_col5 = a_col5(x_k_plus_1);
    A_col6 = a_col6(x_k_plus_1);

    L = lambda_k_plus_2; % for simplicity

       KEEP_DIMS = [
    sum( A_col1(:,:).*L(:,:)); 
    sum( A_col2(:,:).*L(:,:));
    sum( A_col3(:,:).*L(:,:));
    sum( A_col4(:,:).*L(:,:));
    sum( A_col5(:,:).*L(:,:));
    sum( A_col6(:,:).*L(:,:));
    ]; % keep_dims must be 6x # train
    lambda_k_plus_1_target = Attitude_Q * (x_k_plus_1) + discount*KEEP_DIMS;
    Last_W = Attitude_W;
    % Least squares to update network weights
    Attitude_W = (LHS * LHS')\(LHS * lambda_k_plus_1_target');
    if isnan(Attitude_W)
        fprintf('Divergence in trainig \n')
        break
    end

    % Check for convergence
    
    %error(:, :) = Attitude_W' * LHS - lambda_k_plus_1_target;
    weight_plot(:,i) = reshape(Attitude_W.',1,[]);
    error(:, :) = Attitude_W - Last_W;
    error_test = mae(error(:,:));
    if mod(i, 100) == 0
        fprintf('At: %g iterations, the MAE is %f \n', i,error_test)
    end

    if error_test< threshold
        fprintf('Weights converged, mae = %f \n', mae(error(:,:)))
        break
    end
end
fprintf("finished training")

%% Simualtion

figure;
for k = 1:size(weight_plot,1) 
    hold on;
    plot(1:size(weight_plot,2) ,weight_plot(k,:)); 
end
xlabel('Iterations');
ylabel('Weights');
xlim([0 5000]); 


Training_time = toc;
x1 = 1.2*[rand(3,1);0;0;0];
x2 = [rand(3,1);0;0;0];
x3 = [rand(3,1);0;0;0];
x4 = [rand(3,1);0;0;0]; 
% x5 = [rand(3,1);0;0;0];
% x6 = [rand(3,1);0;0;0]; 
% x7 = [rand(3,1);0;0;0]; 
% x8 = [rand(3,1);0;0;0];
% x9 = [rand(3,1);0;0;0]; 

r = [0;0;0;0;0;0];

for i = 1:N
    time(i) = (i-1)*dt;
    r(:, i) = time(i)*0;
    u1(:,i) = -Attitude_R^-1 * Attitude_G(x1(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x1(:,i)-r(:, i));
    u2(:,i) = -Attitude_R^-1 * Attitude_G(x2(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x2(:,i)-r(:, i));
    u3(:,i) = -Attitude_R^-1 * Attitude_G(x3(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x3(:,i)-r(:, i));
    u4(:,i) = -Attitude_R^-1 * Attitude_G(x4(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x4(:,i)-r(:, i));
%     u5(:,i) = -Attitude_R^-1 * Attitude_G(x5(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x5(:,i)-r(:, i));
%     u6(:,i) = -Attitude_R^-1 * Attitude_G(x6(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x6(:,i)-r(:, i));
%     u7(:,i) = -Attitude_R^-1 * Attitude_G(x7(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x7(:,i)-r(:, i));
%     u8(:,i) = -Attitude_R^-1 * Attitude_G(x8(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x8(:,i)-r(:, i));
%     u9(:,i) = -Attitude_R^-1 * Attitude_G(x9(:,i)-r(:, i))' * Attitude_W(:,:)' * basis_2(x9(:,i)-r(:, i));
    x1(:, i+1) = Attitude_F(x1(:,i)) + Attitude_G(x1(:,i)) * u1(:,i);
    x2(:, i+1) = Attitude_F(x2(:,i)) + Attitude_G(x2(:,i)) * u2(:,i);
    x3(:, i+1) = Attitude_F(x3(:,i)) + Attitude_G(x3(:,i)) * u3(:,i);
    x4(:, i+1) = Attitude_F(x4(:,i)) + Attitude_G(x4(:,i)) * u4(:,i);
%     x5(:, i+1) = Attitude_F(x5(:,i)) + Attitude_G(x5(:,i)) * u5(:,i);
%     x6(:, i+1) = Attitude_F(x6(:,i)) + Attitude_G(x6(:,i)) * u6(:,i);
%     x7(:, i+1) = Attitude_F(x7(:,i)) + Attitude_G(x7(:,i)) * u7(:,i);
%     x8(:, i+1) = Attitude_F(x8(:,i)) + Attitude_G(x8(:,i)) * u8(:,i);
%     x9(:, i+1) = Attitude_F(x9(:,i)) + Attitude_G(x9(:,i)) * u9(:,i);
end
figure(1)
    
subplot(3,1,1); plot(...
    time, r(1, 1:length(time)), 'r--',...
    time, x1(1, 1:length(time)),...
    time, x2(1, 1:length(time)),...
    time, x3(1, 1:length(time)),...
    time, x4(1, 1:length(time)))%,...
%     time, x5(1, 1:length(time)),...
%     time, x6(1, 1:length(time)),...
%     time, x7(1, 1:length(time)),...
%     time, x8(1, 1:length(time)),...
%     time, x9(1, 1:length(time)))
xlabel('Time (sec)' )
ylabel('error in pitch')
subplot(3,1,2); plot( ...
    time, r(2, 1:length(time)), 'r--',...
    time, x1(2, 1:length(time)), ...
    time, x2(2, 1:length(time)), ...
    time, x3(2, 1:length(time)), ...
    time, x4(2, 1:length(time)))%, ...
%     time, x5(2, 1:length(time)),...
%     time, x6(2, 1:length(time)),...
%     time, x7(2, 1:length(time)),...
%     time, x8(2, 1:length(time)),...
%     time, x9(2, 1:length(time)))
xlabel('Time (sec)' )
ylabel('error in roll')
subplot(3,1,3); plot( ...
    time, r(3, 1:length(time)), 'r--',...
    time, x1(3, 1:length(time)), ...
    time, x2(3, 1:length(time)), ...
    time, x3(3, 1:length(time)), ...
    time, x4(3, 1:length(time)))%, ...
%     time, x5(3, 1:length(time)),...
%     time, x6(3, 1:length(time)),...
%     time, x7(3, 1:length(time)),...
%     time, x8(3, 1:length(time)),...
%     time, x9(3, 1:length(time)))
xlabel('Time (sec)' )
ylabel('error in yaw')

figure(2)
subplot(3,1,1); plot(...
    time, r(4, 1:length(time)), 'r--',...
    time, x1(4, 1:length(time)),...
    time, x2(4, 1:length(time)),...
    time, x3(4, 1:length(time)),...
    time, x4(4, 1:length(time)))%,...
%     time, x5(4, 1:length(time)),...
%     time, x6(4, 1:length(time)),...
%     time, x7(4, 1:length(time)),...
%     time, x8(4, 1:length(time)),...
%     time, x9(4, 1:length(time)))
xlabel('Time (sec)' )
ylabel('error in pitch rate (rad/sec)')
subplot(3,1,2); plot( ...
    time, r(5, 1:length(time)), 'r--',...
    time, x1(5, 1:length(time)), ...
    time, x2(5, 1:length(time)), ...
    time, x3(5, 1:length(time)), ...
    time, x4(5, 1:length(time)))%, ...
%     time, x5(5, 1:length(time)),...
%     time, x6(5, 1:length(time)),...
%     time, x7(5, 1:length(time)),...
%     time, x8(5, 1:length(time)),...
%     time, x9(5, 1:length(time)))
xlabel('Time (sec)' )
ylabel('error in roll rate (rad/sec)')
subplot(3,1,3); plot( ...
    time, r(6, 1:length(time)), 'r--',...
    time, x1(6, 1:length(time)), ...
    time, x2(6, 1:length(time)), ...
    time, x3(6, 1:length(time)), ...
    time, x4(6, 1:length(time)))%, ...
%     time, x5(6, 1:length(time)),...
%     time, x6(6, 1:length(time)),...
%     time, x7(6, 1:length(time)),...
%     time, x8(6, 1:length(time)),...
%     time, x9(6, 1:length(time)))
xlabel('Time (sec)' )
ylabel('error in yaw rate (rad/sec)')

figure
subplot(3,1,1); plot(...
    time, u1(1,1:length(time)),...
    time, u2(1,1:length(time)),...
    time, u3(1,1:length(time)),...
    time, u4(1,1:length(time)))%,...
%     time, u5(1,1:length(time)),...
%     time, u6(1,1:length(time)),...
%     time, u7(1,1:length(time)),...
%     time, u8(1,1:length(time)),...
%     time, u9(1,1:length(time)))
xlabel('Time (sec)' )
ylabel('u1')
subplot(3,1,2); plot(...
    time, u1(2,1:length(time)),...
    time, u2(2,1:length(time)),...
    time, u3(2,1:length(time)),...
    time, u4(2,1:length(time)))%,...
%     time, u5(2,1:length(time)),...
%     time, u6(2,1:length(time)),...
%     time, u7(2,1:length(time)),...
%     time, u8(2,1:length(time)),...
%     time, u9(2,1:length(time)))
xlabel('Time (sec)' )
ylabel('u2')
subplot(3,1,3); plot(...
    time, u1(3,1:length(time)),...
    time, u2(3,1:length(time)),...
    time, u3(3,1:length(time)),...
    time, u4(3,1:length(time)))%,...
%     time, u5(3,1:length(time)),...
%     time, u6(3,1:length(time)),...
%     time, u7(3,1:length(time)),...
%     time, u8(3,1:length(time)),...
%     time, u9(3,1:length(time)))
xlabel('Time (sec)' )
ylabel('u3')

save att.mat

% figure
% grid on
% hold on
% plot3(r(1,:), r(2,:), -r(3,:), '--', 'Linewidth', 1.5)
% plot3(x1(1,:), x1(2,:), -x1(3,:), 'Linewidth', 1.5)
% plot3(x2(1,:), x2(2,:), -x2(3,:), 'Linewidth', 1.5)
% plot3(x3(1,:), x3(2,:), -x3(3,:), 'Linewidth', 1.5)
% plot3(x4(1,:), x4(2,:), -x4(3,:), 'Linewidth', 1.5)
% plot3(x5(1,:), x5(2,:), -x5(3,:), 'Linewidth', 1.5)
% plot3(x6(1,:), x6(2,:), -x6(3,:), 'Linewidth', 1.5)
% plot3(x7(1,:), x7(2,:), -x7(3,:), 'Linewidth', 1.5)
% plot3(x8(1,:), x8(2,:), -x8(3,:), 'Linewidth', 1.5)
% plot3(x9(1,:), x9(2,:), -x9(3,:), 'Linewidth', 1.5)
% title('3D Trajectory')
% xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
% legend(["Reference trajectory", "SNAC"]);
% fprintf('required time for training = %g sec\n', Training_time)




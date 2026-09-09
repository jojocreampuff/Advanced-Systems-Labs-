%% using just attitude RLS SNAC controlller
clear; close all; clc;
load("SNAC_GD_att_2.mat","GD_SNAC_MF","params")
plot_training(GD_SNAC_MF.t',GD_SNAC_MF.err,GD_SNAC_MF.theta_hist',"RLS Training",10)
W_att = GD_SNAC_MF.W_final;
R_att = params.R;

%% standard IC
T = 15;
dt = 0.001;
N = T/dt;
time = 0:dt:(T-dt);
Ix = 0.2;
Iy = 0.3;
Iz = 0.4;
att_G = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];
att_Gz = [att_G;
          zeros(size(att_G))];
m = 1; grav = 9.81;
U = zeros(4, length(time));  % Control input trajectory
X_ref = zeros(12, length(time));  % Reference trajectory
r_yaw = pi/4*ones(1,N); 
X = zeros(12,length(time));
X(:,1) = [0; 0; 0; zeros(9,1)];
% X(9,1) = pi/4;
% X(:,1) = [5
% 0
% -5
% -0.000624999986875707
% 2.49999989583333
% 0
% 5.09428683952954e-16
% -5.10203936918714e-16
% 0.785398163397448
% -1
% -1
% 1];

A_pos = [0  0   0   1   0   0   
        0   0   0   0   1   0   
        0   0   0   0   0   1   
        0   0   0   0   0   0  
        0   0   0   0   0   0   
        0   0   0   0   0   0];

B_pos = [0  0   0  
        0   0   0
        0   0   0
        1   0   0
        0   1   0
        0   0   1];
A_att = [0  0   0   1   0   0   
        0   0   0   0   1   0   
        0   0   0   0   0   1   
        0   0   0   0   0   0  
        0   0   0   0   0   0  
        0   0   0   0   0   0];

B_att = [0   0   0 
         0   0   0 
         0   0   0 
         1/Ix    0   0
         0   1/Iy    0
         0    0      1/Iz];

Q_pos = diag([1,1,1,1,1,1])*50;
R_pos = diag([1,1,1])*10; 
[K_pos,~,~] = lqr(A_pos,B_pos,Q_pos,R_pos);
Q_att = diag([10,10,10,1,1,1]);
[K_att,~,~] = lqr(A_att,B_att,Q_att,R_att);


ref = @(t) [5*cos(0.5*t);       % reference_x
             5*sin(0.5*t);     % reference_y
             -5];
             % 1*cos(2*.5*t)-5];      % reference_z
%  ref =    [amp*cos(ohm*t);          amp*sin(ohm*t);         amp_z*sin(2*ohm*t)-z
%           -amp*ohm*sin(ohm*t);      amp*ohm*cos(ohm*t);     2*amp_z*ohm*cos(2*ohm*t)];

for j = 1:length(time)
    t = time(j);
    r_initial(:,j) = ref(t);
end

r_smooth = [r_initial; discrete_deriv(r_initial,dt)];

tic
omega_d_dot_filt = zeros(3,1);
for i = 1:length(time)

    %% LQR position control
    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);
    U_pos = -K_pos * (pos_error(:,i)); % lqr_control

    [ft, r_pitch, r_roll] = borna_sys_solve(U_pos(1), U_pos(2), U_pos(3) - grav, r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    omega_d_dot_raw = deriv(angles,i,dt);
    alpha = 0.9;  % filter factor (0.8-0.98)
    omega_d_dot = alpha * omega_d_dot_filt + (1-alpha) * omega_d_dot_raw;
    omega_d_dot_filt = omega_d_dot;
    X_ref(7:12, i) = [angles(:,i); omega_d_dot_filt];
    % X_ref(7:12, i) = [angles(:,i); deriv(angles,i,dt)];

    %% Online SNAC LS att control 
    Att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = -0.5*R_att^-1*att_Gz'*W_att'*PHI(Att_error(:,i), X_ref(7:12, i)); % learned att control
    % torques(:,i) = -K_att * (Att_error(:,i)); % lqr control

    U(:,i) = [ft; torques(:,i)];

    %% Update Dynamics using Euler integration
    f_x = Full_f_225(X(:, i),grav,Ix,Iy,Iz);
    g_x = Full_g_225(X(:, i), m,Ix,Iy,Iz);

    % State update using Euler integration
    if i < length(time)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end

end

single_loop_final_time = toc;
single_loop_results.X = X;
single_loop_results.U = U;
single_loop_results.error = [pos_error; Att_error];
single_loop_results.r_initial = r_initial;
single_loop_results.simtime = single_loop_final_time;
single_loop_error = single_loop_results.error;
single_loop_tracking_error = (single_loop_error(1,:).^2 + single_loop_error(2,:).^2 + single_loop_error(3,:).^2).^0.5;
single_loop_tracking_error_sum = dt*sum(single_loop_tracking_error,2);
single_loop_att_tracking_error = (single_loop_error(7,:).^2 + single_loop_error(8,:).^2 + single_loop_error(9,:).^2).^0.5;
rmse_att = sqrt(mean(sum(single_loop_error(7:9,:),1)).^2)

save("z_GD_attitude_tracking")
% saveFigures("RLS_Drone_att5")
figure;
hold on;
plot(time(1:end-1), single_loop_att_tracking_error(1,1:N-1), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Tracking Error [rad]');
title('L2 Norm Tracking Error');
grid on;

% 
% figure;
% plot(time, U, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Control Inputs');
% legend('ft', 'u2', 'u3', 'u4');
% grid on;
% title('Control Inputs Over Time');

% figure;
% titles = {'x', 'y', 'z', 'u', 'v', 'w', '$\phi$', '$\theta$', '$\psi$', 'p', 'q', 'r'};
% for i = 1:12
%     subplot(4,3,i);
%     plot(time, X(i,:), 'b', 'LineWidth', 1.5); hold on;
%     plot(time, X_ref(i,:), 'r--', 'LineWidth', 1.5);
%     xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i}, 'Interpreter', 'latex');
% 
%     grid on;
% end
% legend('O-SNAC', 'Reference');
% sgtitle('Reference in $\Phi(e)$', 'Interpreter', 'latex');


% figure;
% titles = {'$\phi$', '$\theta$', '$\psi$', 'p', 'q', 'r'};
% for i = 7:12
%     subplot(2,3,i-6);
%     plot(time, X(i,:), 'b', 'LineWidth', 1.5); hold on;
%     plot(time, X_ref(i,:), 'r--', 'LineWidth', 1.5);
%     xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i-6}, 'Interpreter', 'latex');
% 
%     grid on;
% end
% legend("SNAC Attitude", 'Reference');
% sgtitle("RLS Attitude Tracking", Interpreter="latex");

% figure; hold on; grid on; box on;
% set(gca, 'GridLineStyle', ':');
% titles = {'$e_{\phi} $', '$e_{\theta}$', '$e_{\psi}$', '$e_{p}$', '$e_{q}$', '$e_{r}$'};
% for i = 7:12
%     subplot(2,3,i-6);
%     plot(time, X(i,:)- X_ref(i,:), 'r', 'LineWidth', 1.5); hold on;
%     xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i-6}, 'Interpreter', 'latex');
%     grid on;
% end

figure;
titles = {'$\phi$', '$\theta$', '$\psi$'};
for i = 7:9
    subplot(3,1,i-6);
    hold on; grid on; box on;
    set(gca, 'GridLineStyle', ':');
    plot(time, X(i,:), 'r', 'LineWidth', 1.5); hold on;
    plot(time, X_ref(i,:), 'g--', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i-6}, 'Interpreter', 'latex');
    grid on;
    legend(titles{i-6}, '$r(t)$', 'Interpreter', 'latex');
end


figure; 
titles = { 'p', 'q', 'r'};
for i = 10:12
    subplot(3,1,i-9);
    hold on; grid on; box on;
    set(gca, 'GridLineStyle', ':');
    plot(time, X(i,:), 'r', 'LineWidth', 1.5);
    plot(time, X_ref(i,:), 'g--', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i-9}, 'Interpreter', 'latex');
    grid on;
    legend(titles{i-9}, '$r(t)$', 'Interpreter', 'latex');
end

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(time, X(7,:)- X_ref(7,:));
plot(time, X(8,:)- X_ref(8,:));
plot(time, X(9,:)- X_ref(9,:));
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel("(rad)", 'Interpreter', 'latex');
legend("$e_{\phi}$","$e_{\theta}$","$e_{\psi}$", 'Interpreter','latex')
grid on;

figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(time, X(10,:)- X_ref(10,:));
plot(time, X(11,:)- X_ref(11,:));
plot(time, X(12,:)- X_ref(12,:));
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel("(rad/s)", 'Interpreter', 'latex');
legend("$e_{p}$","$e_{q}$","$e_{r}$", 'Interpreter','latex')
grid on;

figure; 
subplot(2,1,1)
hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(time, X(7,:)- X_ref(7,:));
plot(time, X(8,:)- X_ref(8,:));
plot(time, X(9,:)- X_ref(9,:));
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel("(rad)", 'Interpreter', 'latex');
legend("$e_{\phi}$","$e_{\theta}$","$e_{\psi}$", 'Interpreter','latex')
subplot(2,1,2)
hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
plot(time, X(10,:)- X_ref(10,:));
plot(time, X(11,:)- X_ref(11,:));
plot(time, X(12,:)- X_ref(12,:));
xlabel('Time (s)', 'Interpreter', 'latex'); ylabel("(rad/s)", 'Interpreter', 'latex');
legend("$e_{p}$","$e_{q}$","$e_{r}$", 'Interpreter','latex')


figure; hold on; grid on; box on;
set(gca, 'GridLineStyle', ':');
titles = {'$e_{p}$', '$e_{q}$', '$e_{r}$'};
for i = 10:12
    subplot(3,1,i-9);
    plot(time, X(i,:)- X_ref(i,:), 'r', 'LineWidth', 1.5); hold on;
    xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i-9}, 'Interpreter', 'latex');
    grid on;
end

figure; hold on; grid on;
set(gca, 'GridLineStyle', ':');
plot3(X(7,:),   X(8,:), X(9,:), 'Color', [1 0 0],"LineStyle","-", 'LineWidth', 1);
plot3(X_ref(7,:), X_ref(8,:), X_ref(9,:),"Color","g","LineStyle","--","LineWidth",1);
xlabel("$\phi$ (rad)", Interpreter="latex"); ylabel("$\theta$ (rad)", Interpreter="latex"); zlabel("$\psi$ (rad)", Interpreter="latex");
legend("$X_{a}(t)$","$r_{a}(t)$",'Location','best', Interpreter="latex");
% title("RLS Attitude Tracking", Interpreter="latex");

figure; hold on; grid on;
set(gca, 'GridLineStyle', ':');
plot3(X(1,:), X(2,:), -X(3,:), 'r', 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:1000:length(X(1,:)), 'MarkerSize', 5)
plot3(r_initial(1,:), r_initial(2,:), -r_initial(3,:), '--', 'Color', [0 1 0], 'Linewidth', 1.5)
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
legend(["$X_{p}(t)$","$r_{p}(t)$"], 'Interpreter', 'latex');

function [ft, pitch, roll] = borna_sys_solve(u1, u2, u3, psi, m) % u1 = ux , u2 = uy , u3 = uz - g  
    g = 9.81;
    a = sin(psi);
    b = cos(psi);
    A = u1 / (u3);
    B = u2 / (u3);
    C = (a*A - b*B) / ( (a^2) + (b^2) );
    roll = atan( (B + b*C) / a );
    pitch = atan( C * cos(roll) );
    % saftey limit threshold on desired pitch and roll
    max_angle = pi/4;
    if pitch>max_angle
        pitch = max_angle;
    elseif pitch<-max_angle
        pitch = -max_angle;
    end
    if roll>max_angle
        roll = max_angle;
    elseif roll<-max_angle
        roll = -max_angle;
    end
    ft = ( m / (cos(pitch)*cos(roll)) ) * (-u3); 
    % 
    % if ft > 2*m*g
    %     ft = 2*m*g;
    % end
end

function pqr = deriv(angles, i, dt)
if i == 1
    pqr = ((angles(:, i) - 0)/dt);
elseif i > 1
    pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
end
% Limit pqr to be between -5 and 5
pqr(pqr >1) = 1;
pqr(pqr < -1) = -1;
end

function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end

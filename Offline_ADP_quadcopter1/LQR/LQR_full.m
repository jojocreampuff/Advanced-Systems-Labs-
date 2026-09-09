%% LQR for the SNAC paper

clear; clc; close all;
diverged = 0; 
g = 9.81;
dt = 0.001;
tf = 30;
Ix = 0.3;   % moment of inertia (kg*m^2)
Iy = 0.4;   % moment of inertia (kg*m^2)
Iz = 0.5;   % moment of inertia (kg*m^2)
m = 1;

% reference is just for the first 6 states
F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
    (1-exp(-0.01*t))*9.81*sin(0.2*t);
    -.1*t
    (sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000
    (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5
    -.1];

%% linear drone
A = [0  0   0   1   0   0   0   0   0   0   0   0
    0   0   0   0   1   0   0   0   0   0   0   0
    0   0   0   0   0   1   0   0   0   0   0   0
    0   0   0   0   0   0   0   -g  0   0   0   0
    0   0   0   0   0   0   g   0   0   0   0   0
    0   0   0   0   0   0   0   0   0   0   0   0
    0   0   0   0   0   0   0   0   0   1   0   0
    0   0   0   0   0   0   0   0   0   0   1   0
    0   0   0   0   0   0   0   0   0   0   0   1
    0   0   0   0   0   0   0   0   0   0   0   0
    0   0   0   0   0   0   0   0   0   0   0   0
    0   0   0   0   0   0   0   0   0   0   0   0    ];

B = [0  0   0   0
    0   0   0   0
    0   0   0   0
    0   0   0   0
    0   0   0   0
    1/m   0   0   0
    0   0   0   0
    0   0   0   0
    0   0   0   0
    0   1/Ix    0   0
    0   0   1/Iy    0
    0   0    0      1/Iz];

Q = 1*diag([1e1,1e1,1e1,1e1,1e1,1e1,1,1,1,1,1,1]);
R = 1*diag([1e0,1e0,1e0,1e0]);

[K_LQR,S,CLP] = lqr(A,B,Q,R);

%% Simulation loop
T = 0:dt:tf;
X = zeros(12, length(T)); % state trajectory
X(1:2,1) = [1;-1];
X_nonlinear = zeros(12, length(T)); % Nonlinear state trajectory
X_nonlinear(1:2,1) = [1;-1];
U = zeros(4, length(T)); % Control input trajectory
U_nonlinear = zeros(4, length(T));  % control input trajectory
X_ref = zeros(12, length(T)); % reference trajectory

for i = 1:length(T)
    t = T(i);

    % Compute reference trajectory
    X_ref(1:6, i) = F_r(t);
    X_ref(7:12, i) = 0;  % The last 6 states are zero

    U(:, i) = -K_LQR * (X(:, i) - X_ref(:, i));
    U_nonlinear(:,i) = -K_LQR * (X_nonlinear(:, i) - X_ref(:, i));

    % Update Nonlinear Dynamics using Euler integration
    f_x = Full_f_225(X_nonlinear(:, i),g,Ix,Iy,Iz);
    g_x = Full_g_225(X_nonlinear(:, i), m,Ix,Iy,Iz);

    % State update using Euler integration
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (A * X(:, i) + B * U(:, i));
        X_nonlinear(:, i+1) = X_nonlinear(:, i) + dt * (f_x + g_x * U_nonlinear(:, i));
        if isnan(norm(X_nonlinear(:,i),2)) == 1
            fprintf("Diverged\n")
            diverged = 1;
            break
        end


    end

end
if diverged ==0

    %% Plot results
    %% Plot Results - Compare Linear and Nonlinear Dynamics
    % figure;
    % titles = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'p', 'q', 'r'};
    % for i = 1:12
    %     subplot(4,3,i);
    %     plot(T, X(i,:), 'b', 'LineWidth', 1.5); hold on;
    %     plot(T, X_nonlinear(i,:), 'g', 'LineWidth', 1.5);
    %     plot(T, X_ref(i,:), 'r--', 'LineWidth', 1.5);
    %     xlabel('Time (s)'); ylabel(titles{i});
    %     legend('Linear Model', 'Nonlinear Model', 'Reference');
    %     grid on;
    % end
    % sgtitle('LQR Tracking: Linear vs Nonlinear Dynamics');

    % figure;
    % plot(T, U, 'LineWidth', 1.5);
    % xlabel('Time (s)');
    % ylabel('Control Inputs');
    % legend('u1', 'u2', 'u3', 'u4');
    % grid on;
    % title('Control Inputs Over Time');
    %
    % figure;
    % grid on
    % hold on
    % plot3(X_ref(1,:), X_ref(2,:), -X_ref(3,:), '--', 'Linewidth', 1.5)
    % plot3(X(1,:), X(2,:), -X(3,:), 'Linewidth', 1.5)
    % title('3D Trajectory linear Dynamics')
    % xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
    % legend(["Reference trajectory", "LQR"]);

    figure;
    grid on
    hold on
    plot3(X_ref(1,:), X_ref(2,:), -X_ref(3,:), '--', 'Linewidth', 1.5)
    plot3(X_nonlinear(1,:), X_nonlinear(2,:), -X_nonlinear(3,:), 'Linewidth', 1.5)
    title('3D Trajectory non-linear Dynamics')
    xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
    legend(["Reference trajectory", "LQR"]);
end
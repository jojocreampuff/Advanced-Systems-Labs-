clear; close all; clc;
load("holy_crap_it_worked3.mat", "W", "R","Q")
W_att = W;
R_att = R;
Q_att = Q;

load("pos_controller_good_mae_low.mat", "W")
W_pos = W;
R_pos = R;
Q_pos = Q;
%% linear drone
% pos A and B
% the control from this lqr will be ax ay az same as SNAC
% the system solver will convert this to the desired angles and ft
f_pos = @(x) [x(4); x(5); x(6); 0; 0; 0];
g_pos = @(x) [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
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

%% Make sure that these are the same as in the snac training files
[K_pos,S,CLP] = lqr(A_pos,B_pos,Q_pos,R_pos);
tic
%% standard IC
T = 50;
dt = 0.01;
N = T/dt;
time = dt:dt:T;
g = 9.81;
tf = 50;
Ix = 0.3;   % Moment of inertia (kg*m^2)
Iy = 0.4;
Iz = 0.5;
m = 1;
U = zeros(4, length(time));  % Control input trajectory
X_ref = zeros(12, length(time));  % Reference trajectory
r_yaw = pi/2*ones(1,N); 
% cost plotting 
instant_cost_pos = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_pos = zeros(1, length(T)); % Integrated cost
instant_cost_att = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_att = zeros(1, length(T)); % Integrated cost

% Reference trajectory function
F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t;];
% F_r = @(t) [5*cos(0.20*t);       % reference_x
%              5*sin(0.20*t);     % reference_y
%               2*sin(0.80*t)-5];      % reference_z

%% standard IC
X = zeros(12,length(time));
X(:,1) = [5; 5; 0
1.16
-5.10
0.79
0.13
-0.54
pi/2
-0.07
-0.02
0.5];

for j = 1:length(time)
    t = time(j);
    r_initial(:,j) = F_r(t);
end

smooth_r_position = smooth(r_initial, X(1:3), X(4:6), T);
r_smooth = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];
r_smooth = [r_initial; discrete_deriv(r_initial,dt)];


for i = 1:length(time)

    %% position control (LQR)
    % X_ref(1:6, i) = r_smooth(:,i);
    % pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);
    % U_pos = -K_pos * (pos_error(:,i)) - [0; 0; g];
    % [ft, r_pitch, r_roll] = borna_sys_solve(U_pos(1), U_pos(2), U_pos(3), r_yaw(i), m);
    % angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    % X_ref(7:12, i) = [angles(:,i); deriv(angles,i,dt)];
    %% Postion Control (SNAC)
    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);
    U_pos = -0.5*R_pos^-1*g_pos(X(1:6,i))'*grad_Basis_func_pos(pos_error(:,i))'*W_pos' - [0; 0; g];
    [ft, r_pitch, r_roll] = borna_sys_solve(U_pos(1), U_pos(2), U_pos(3), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); deriv(angles,i,dt)];

    %% att control (SNAC)
    Att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques = -0.5*R_att^-1*att_G(X(7:12,i))'*grad_Basis_func_att(Att_error(:,i))'*W_att';
    %% Att Control (LQR)
    % Att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    % torques = -K * (Att_error(:,i));

    U(:,i) = [ft; torques];

    %% Update Nonlinear Dynamics using Euler integration
    f_x = Full_f_225(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g_225(X(:, i), m,Ix,Iy,Iz);

    
    % State update using Euler integration
    if i < length(time)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end
    instant_cost_pos(i) = pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos' * R_pos * U_pos;
    instant_cost_att(i) = Att_error(:,i)' * Q_att * Att_error(:,i) + torques' * R_att * torques;
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + dt*instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + dt*instant_cost_att(i);
   end

end

SNAC_final_time = toc;
SNAC_results.X = X;
SNAC_results.U = U;
SNAC_results.error = [pos_error;Att_error];
SNAC_results.r_initial = r_initial;
SNAC_results.inscost = [instant_cost_pos;instant_cost_att];
SNAC_results.cumcost = [cumulative_cost_pos;cumulative_cost_att];
SNAC_results.simtime = SNAC_final_time;
SNAC_error = SNAC_results.error;
SNAC_tracking_error = (SNAC_error(1,:).^2 + SNAC_error(2,:).^2 + SNAC_error(3,:).^2).^0.5;
SNAC_tracking_error_sum = dt*sum(SNAC_tracking_error,2)
SNAC_c_cost_total = sum(SNAC_results.cumcost, 1);
cost = SNAC_c_cost_total(end-1);

save("single_critic_r2.mat","SNAC_tracking_error","time")

figure;
hold on;
plot(time(1:end-1), SNAC_tracking_error(1,1:N-1), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Tracking Error [m]');
title('L2 Norm Tracking Error');
grid on;

figure;
plot(time, U, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Inputs');
legend('ft', 'u2', 'u3', 'u4');
grid on;
title('Control Inputs Over Time');

figure;
titles = {'x', 'y', 'z', 'u', 'v', 'w', '$\phi$', '$\theta$', '$\psi$', 'p', 'q', 'r'};
for i = 1:12
    subplot(4,3,i);
    plot(time, X(i,:), 'b', 'LineWidth', 1.5); hold on;
    plot(time, X_ref(i,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i}, 'Interpreter', 'latex');
    
    grid on;
end
legend('O-SNAC', 'Reference');
sgtitle('LQR X SNAC Tracking: All states');

figure;
grid on
hold on
plot3(r_initial(1,:), r_initial(2,:), -r_initial(3,:), '--', 'Color', [0 1 0], 'Linewidth', 1.5)
plot3(X(1,:), X(2,:), -X(3,:), 'r', 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:1000:length(X(1,:)), 'MarkerSize', 5)
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
legend(["$r(t)$", "O-SNAC"], 'Interpreter', 'latex');

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
    if pitch>pi/4
        pitch = pi/4;
    elseif pitch<-pi/4
        pitch = -pi/4;
    end
    if roll>pi/4
        roll = pi/4;
    elseif roll<-pi/4
        roll = -pi/4;
    end
    ft = ( m / (cos(pitch)*cos(roll)) ) * (-u3); 

    if ft > 2*m*g
        ft = 2*m*g;
    end
end

function pqr = deriv(angles, i, dt)
if i == 1
    pqr = ((angles(:, i) - 0)/dt);
elseif i > 1
    pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
end
% Limit pqr to be between -5 and 5
pqr(pqr > 1) = 1;
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

% saveFigures("test1")
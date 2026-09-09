%% PID main
clear; clc; close all;
rng(5,"twister")
g = 9.81;
grav = g;
dt = 0.004;
tf = 50;
Ix = 0.3;   % Moment of inertia (kg*m^2)
Iy = 0.4;
Iz = 0.5;
m = 1;
% cost function parameters
%% Make sure that these are the same as in the snac training files
Position_Q = diag([1000,1000,1000,1000,1000,1000]);
Position_R = diag([1,1,1])*50; % SITL has too much control input
Q_pos = Position_Q;
R_pos = Position_R;

Attitude_Q = diag([10,10,10,1,1,1])*1000;
Attitude_R = diag([1,1,1])*50;
Q_att = Attitude_Q;
R_att = Attitude_R;

% Load the PID gains
load('PID_Gains.mat');

% Reference trajectory function
F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t;];

%% **Simulation Initialization**
T = 0:dt:tf-dt;
N = tf/dt;
X = zeros(12, length(T));  % Nonlinear state trajectory
%% wind case IC
% X(:,1) = [5; 5; 0
% zeros(9,1)];
%% standard IC
X(:,1) = [5; 5; 0
-1.161992909093235
-5.108801409368473
0.790206167774050
0.1355
-0.542388992513929
pi/2
-0.0712468654804163
-0.00259254398073884
0.516408016763246];
%% failure mode IC
% X(:,1) = [100; 100; 100
% -10
% 10
% -10
% pi/4
% -pi/4
% pi
% -pi/4
% pi/4
% pi/4];
%% Failure IC
U = zeros(4, length(T));  % Control input trajectory
X_ref = zeros(12, length(T));  % Reference trajectory
r_yaw = ones(1,N);

%% Test 2: disturbance and uncertainty
add_wind = 0;
add_sensor_noise = 0;
add_actuator_noise = 0;
add_saturation = 0; % logical value
load("wind_simulation.mat")
L = 0.2;
drag = 0.02; % (drag coef)
max_Ft = 2*m*grav;
Tmin = 0;
Tmax = max_Ft/4;
Max_torque = Tmax*L*2;

sensor_uncert = [0.02, 0.02, 0.05, 0.05, 0.05, 0.05, 0.15, 0.15, 0.5, 0.005, 0.005, 0.005]';
actuator_uncert = [0.1*max_Ft, 0.01*Max_torque, 0.01*Max_torque, 0.02*Max_torque]';

% cost plotting 
instant_cost_pos = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_pos = zeros(1, length(T)); % Integrated cost
instant_cost_att = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_att = zeros(1, length(T)); % Integrated cost

% Initialize error terms for PID controllers
pos_error = zeros(3,N);
vel_error = zeros(3,N);
integral_pos = zeros(3,1);
integral_vel = zeros(3,1);
integral_att = zeros(3,1);
integral_ang = zeros(3,1);
previous_error_pos = zeros(3,1);
previous_error_vel = zeros(3,1);
previous_error_att = zeros(3,1);
previous_error_ang = zeros(3,1);

for j = 1:length(T)
    t = T(j);
    r_initial(:,j) = F_r(t);
end

r_smooth = [r_initial; discrete_deriv(r_initial,dt)];

%% **Simulation Loop**
tic
for i = 1:length(T)
    t = T(i);
    %% add sensor noise
    X(:,i) = add_noise(X(:,i), sensor_uncert, add_sensor_noise);

    %% **1. Compute Position Errors**
    X_ref(1:6, i) = r_smooth(:,i);  % Desired position & velocity
    pos_error(:,i) = X_ref(1:3, i) - X(1:3, i);  % Position error
    % can use reference or velocity PID to get vel error
    % vel_error(:,i) = X_ref(4:6, i) - X(4:6, i);  % Velocity error

    %% **2. Position PID (outputs desired velocities u, v, w)**
    integral_pos = integral_pos + pos_error(:,i) * dt;
    derivative_pos = (pos_error(:,i) - previous_error_pos) / dt;
    previous_error_pos = pos_error(:,i);
    
    desired_vel = [
        PID_Gains.x.Kp * pos_error(1,i) + PID_Gains.x.Ki * integral_pos(1) + PID_Gains.x.Kd*(PID_Gains.x.N / (1+PID_Gains.x.N))* derivative_pos(1);
        PID_Gains.y.Kp * pos_error(2,i) + PID_Gains.y.Ki * integral_pos(2) + PID_Gains.y.Kd*(PID_Gains.y.N / (1+PID_Gains.y.N))* derivative_pos(2);
        PID_Gains.z.Kp * pos_error(3,i) + PID_Gains.z.Ki * integral_pos(3) + PID_Gains.z.Kd*(PID_Gains.z.N / (1+PID_Gains.z.N))*derivative_pos(3);
    ];

    vel_error(:,i) = desired_vel - X(4:6, i);

    %% **3. Velocity PID (outputs desired accelerations a_x, a_y, a_z)**
    integral_vel = integral_vel + vel_error(:,i) * dt;
    derivative_vel = (vel_error(:,i) - previous_error_vel) / dt;
    previous_error_vel = vel_error(:,i);

    desired_accel = [
        PID_Gains.u.Kp * vel_error(1,i) + PID_Gains.u.Ki * integral_vel(1) + PID_Gains.u.Kd *(PID_Gains.u.N / (1+PID_Gains.u.N))* derivative_vel(1);
        PID_Gains.v.Kp * vel_error(2,i) + PID_Gains.v.Ki * integral_vel(2) + PID_Gains.v.Kd *(PID_Gains.v.N / (1+PID_Gains.v.N))* derivative_vel(2);
        PID_Gains.w.Kp * vel_error(3,i) + PID_Gains.w.Ki * integral_vel(3) + PID_Gains.w.Kd *(PID_Gains.w.N / (1+PID_Gains.w.N))* derivative_vel(3);
    ] + [0; 0; g];  % Gravity compensation

    %% Convert acceleration to thrust & desired angles
    [ft, r_pitch, r_roll] = borna_sys_solve(-desired_accel(1), -desired_accel(2), -desired_accel(3), X(9,i), m);
    angles = [r_pitch; r_roll; r_yaw(i)];

    %% **4. Attitude PID (outputs desired angular rates p, q, r)**
    att_error = angles - X(7:9, i);
    integral_att = integral_att + att_error * dt;
    derivative_att = (att_error - previous_error_att) / dt;
    previous_error_att = att_error;

    desired_angular_rates = [
        PID_Gains.pitch.Kp * att_error(1) + PID_Gains.pitch.Ki * integral_att(1) + PID_Gains.pitch.Kd *(PID_Gains.pitch.N / (1+PID_Gains.pitch.N))* derivative_att(1);
        PID_Gains.roll.Kp * att_error(2) + PID_Gains.roll.Ki * integral_att(2) + PID_Gains.roll.Kd *(PID_Gains.roll.N / (1+PID_Gains.roll.N))* derivative_att(2);
        PID_Gains.yaw.Kp * att_error(3) + PID_Gains.yaw.Ki * integral_att(3) + PID_Gains.yaw.Kd * (PID_Gains.yaw.N / (1+PID_Gains.yaw.N))*derivative_att(3);
    ];
    X_ref(7:12,i) = [angles;desired_angular_rates];
    %% **5. Angular Velocity PID (outputs torques)**
    ang_vel_error = desired_angular_rates - X(10:12, i);
    integral_ang = integral_ang + ang_vel_error * dt;
    derivative_ang = (ang_vel_error - previous_error_ang) / dt;
    previous_error_ang = ang_vel_error;

    torques(:,i) = [
        PID_Gains.p.Kp * ang_vel_error(1) + PID_Gains.p.Ki * integral_ang(1) + PID_Gains.p.Kd *(PID_Gains.p.N / (1+PID_Gains.p.N))* derivative_ang(1);
        PID_Gains.q.Kp * ang_vel_error(2) + PID_Gains.q.Ki * integral_ang(2) + PID_Gains.q.Kd *(PID_Gains.q.N / (1+PID_Gains.q.N))* derivative_ang(2);
        PID_Gains.r.Kp * ang_vel_error(3) + PID_Gains.r.Ki * integral_ang(3) + PID_Gains.r.Kd *(PID_Gains.r.N / (1+PID_Gains.r.N))* derivative_ang(3);
    ];
%% saturate the torques to the max torque the drone can produce
    if add_saturation ==1
        for p = 1:3
            if torques(p,i) > Max_torque
                torques(p,i) = Max_torque;
            elseif torques(p,i) < -Max_torque
                torques(p,i) = -Max_torque;
            end
        end
        % solve for the thrust of each motor given those torques    
        T1 = (ft - torques(1,i)/(2*L) - torques(2,i)/(2*L) + torques(3,i)/(4*drag)) / 4;
        T2 = (ft - torques(1,i)/(2*L) + torques(2,i)/(2*L) + torques(3,i)/(4*drag)) / 4;
        T3 = (ft + torques(1,i)/(2*L) - torques(2,i)/(2*L) - torques(3,i)/(4*drag)) / 4;
        T4 = (ft + torques(1,i)/(2*L) + torques(2,i)/(2*L) - torques(3,i)/(4*drag)) / 4;
        each_motor_thrust = [T1;T2;T3;T4];
    
        % saturate each motor thrust to its bounded values
        for j = 1:4
            if each_motor_thrust(j) > Tmax
                each_motor_thrust(j) = Tmax;
            elseif each_motor_thrust(j) < Tmin
                each_motor_thrust(j) = Tmin;
            end
        end
    
    
        if ft>max_Ft
            ft = max_Ft;
        elseif ft<0
            ft = 0;
        end
    end
    %% **6. Apply Control Inputs to Drone Model**
    U(:,i) = [ft; torques(:,i)];

    %% add actuator noise
    U(:,i) = add_noise(U(:,i), actuator_uncert, add_actuator_noise);
    %% **7. Update Nonlinear Drone Dynamics**
    f_x = Full_f_225(X(:, i), g, Ix, Iy, Iz);
    g_x = Full_g_225(X(:, i), m, Ix, Iy, Iz);
    
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end
    pos_err(:,i) = [pos_error(:,i);vel_error(:,i)];
    Att_error(:,i) = [att_error;desired_angular_rates];
    instant_cost_pos(i) = pos_err(:,i)' * Q_pos * pos_err(:,i) + desired_accel' * R_pos * desired_accel;
    instant_cost_att(i) = Att_error(:,i)' * Q_att * Att_error(:,i) + torques(:,i)' * R_att * torques(:,i);
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i) ;
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i) ;
   end

end
PID_final_time = toc;
%% Plot results
% Plot Instantaneous Cost
figure;
plot(T, instant_cost_pos,T,instant_cost_att, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Instantaneous Cost');
title('Instantaneous Cost Over Time');
legend("Position control cost","Attitude Control Cost")
grid on;

% Plot Cumulative Cost
figure;
plot(T, cumulative_cost_pos,T,cumulative_cost_att, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Cumulative Cost');
title('Total Accumulated Cost Over Time');
legend("Position control cost","Attitude Control Cost")
grid on;

figure;
titles = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'p', 'q', 'r'};
for i = 1:12
    subplot(4,3,i);
    plot(T, X(i,:), 'b', 'LineWidth', 1.5); hold on;
    plot(T, X_ref(i,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)'); ylabel(titles{i});
    legend('Nonlinear Model', 'Reference');
    grid on;
end
sgtitle('PID Tracking Nonlinear Dynamics');

figure;
plot(T, U, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Inputs');
legend('ft', 'u2', 'u3', 'u4');
grid on;
title('Control Inputs Over Time');

figure;
grid on
hold on
plot3(X_ref(1,:), X_ref(2,:), -X_ref(3,:), '--', 'Linewidth', 1.5)
plot3(X(1,:), X(2,:), -X(3,:), 'Linewidth', 1.5)
title('3D Trajectory')
xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
legend(["Reference trajectory", "PID"]);
PID_results.X = X;
PID_results.U = U;
PID_results.error = [pos_err;Att_error];
PID_results.inscost = [instant_cost_pos;instant_cost_att];
PID_results.cumcost = [cumulative_cost_pos;cumulative_cost_att];
PID_results.simtime = PID_final_time;

if (add_sensor_noise > 0 || add_actuator_noise > 0) && add_saturation ~=1
    save("PID_compare_workspace_noise.mat","PID_results")
    disp("Saving noise only simulation")
elseif add_saturation == 1
    save("PID_compare_workspace_noise_saturation.mat","PID_results")
    disp("Saving noise and saturation simulation")
else 
    save("PID_compare_workspace.mat","PID_results")
    disp("Saving ideal simulation")
end


%% rememeber to input u1 and u2 and u3 as negative into this function
function [ft, pitch, roll] = borna_sys_solve(u1, u2, u3, psi, m) % u1 = ux , u2 = uy , u3 = uz - g  
    a = sin(psi);
    b = cos(psi);
    A = u1 / (u3);
    B = u2 / (u3);
    C = (a*A - b*B) / ( (a^2) + (b^2)) ;
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
end

function pqr = deriv(angles, i, dt)
if i == 1
    pqr = ((angles(:, i) - 0)/dt);
elseif i > 1
    pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
end
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

function noisy_vector = add_noise(state_vector, mag, noise_percent)
    noise = (noise_percent) * mag .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end

function R = eul2rotm_zyx(phi, theta, psi)
    % Compute trigonometric values
    c_phi = cos(phi);    s_phi = sin(phi);
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi);    s_psi = sin(psi);
    
    % Construct the ZYX rotation matrix
    R = [ c_theta * c_psi,  s_phi * s_theta * c_psi - c_phi * s_psi,  c_phi * s_theta * c_psi + s_phi * s_psi;
          c_theta * s_psi,  s_phi * s_theta * s_psi + c_phi * c_psi,  c_phi * s_theta * s_psi - s_phi * c_psi;
         -s_theta,          s_phi * c_theta,                          c_phi * c_theta ];
end
function [accn_dist, alpha_dist] = wind_f_m(Wxyz, X, rho, Cd, A, r, m, Ix, Iy, Iz)

    R = eul2rotm_zyx( X(7), X(8), X(9) );
    Wxyz_body = R'*Wxyz;
    V_rel = Wxyz_body - X(4:6);
    F_wind = -0.5 * rho * Cd .*diag(A) * (V_rel .* abs(V_rel)); % Compute wind forces
    
    F_wind_motors = repmat(F_wind, 1, 4) + 0.01*randn(3,4);

    M_wind_motors = cross(r, F_wind_motors, 1);
    M_wind = sum(M_wind_motors, 2);  
    
    accn_dist = F_wind/m;
    alpha_dist=    [M_wind(1)/Ix;
                    M_wind(2)/Iy;
                    M_wind(3)/Iz];
end

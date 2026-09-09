clear;close all;clc
%% MPC code for the camparative control methods paper
load('PID_Gains_matt.mat');
rng(5,"twister")
% Add Paths
addpath('Attitude_Controller');
addpath('Position_Controller')
addpath('Data,weights,Networks');
addpath('Functions');
dt = .04;  % outer loop time step
t_f = 51;  % 51 seconds due to predistion horizon, we only take 50 seconds of the simulation
pos_data_points = t_f/dt;
att_data_points = pos_data_points*10;
grav = 9.81;
% drone parameters
Ix = 0.3;   % moment of inertia (kg*m^2)
Iy = 0.4;   % moment of inertia (kg*m^2)
Iz = 0.5;   % moment of inertia (kg*m^2)
mass = 1;

%% Make sure that these are the same as in the snac training files
Position_Q = diag([1000,1000,1000,1000,1000,1000]);
Position_R = diag([1,1,1])*50; % SITL has too much control input
Q_pos = Position_Q;
R_pos = Position_R;

Attitude_Q = diag([10,10,10,1,1,1])*1000;
Attitude_R = diag([1,1,1])*50;
Q_att = Attitude_Q;
R_att = Attitude_R;

% Reference trajectory function
F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t;];

%% wind case IC
X_IC = [5; 5; 0
zeros(9,1)];

T = 0:dt:t_f-dt;

% determining modified reference trajectory based on original trajectory
for i = 1:length(T)
    r_initial(:,i) = F_r(T(i));
end

% smooth_r_position = smooth(r_initial, X_IC(1:3)', X_IC(4:6)', T);
ref_for_cost = [r_initial; discrete_deriv(r_initial,dt)];
r_smooth = r_initial;
% Define Simulation Parameters
time = t_f;                              % Simulation Duration
N = 10;                                 % Time Horizon
K = 200;                                % Num of Gradient Descent Iterations
epsilon = 0.001;                        % Small number for Gradient Descent
alpha = 0.4;                            % Learning Rate for Gradient Descent
decay_rate = 0.4;                       % Decay_rate for Gradient Descent
angular_time_constants = .2;            % Arbitrary Time Constant
% Q  = [10;10;10;1;1;1;1;1;0;10;10];    % Punishment Coefficients
% Qn = [40;40;40;5;5;5;10;10];              % Terminal Punishment Coefficients
Q  = [20;20;20;10;10;10;15;15;0;100;100];    % Punishment Coefficients
Qn = [40;40;40;5;5;5;10;10];              % Terminal Punishment Coefficients
Kp = [3; 3; 1.5];                       % PID Proportional gains
Ki = [0; 0; 0];                         % PID Integral gains
Kd = [4; 4; 1.5];                       % PID Derivative gains
Noise_coefficients = zeros(12,1);       % Random Noise coefficients for States 
U_Noise_coefficients = zeros(4,1);      % Random Noise coefficients for Controls

% Define Cost Functions
J  = @(State_horizon,reference)  sum((State_horizon-reference).^2.*Q,'all')*dt;   % Cost Function
JN = @(State_terminal,reference) sum((State_terminal-reference).^2.*Qn,'all')*dt; % Terminal Cost Function

%Initialization
U = zeros(4,N);                           % Control Matrix
X = zeros(12,1);                          % States
X(:,1)=X_IC;
angle_error_old= 0 ;                      % First Attitude Controller input                 
integral_error = zeros(3,1);              % First Attitude Controller input
first_iteration = true;m = 1;             % Flags 
X_values = zeros(12,time/dt-N);  
X_values(:,1) = X_IC;                       % States History
U_values = zeros(4,time/dt-N);            % Controls History
gradient = zeros(3,N);                    % Controls Gradient Matrix
pos_controls = zeros(3,N);                % MPC Controls
pos_controls(1,:) = 9.8;                  % Initial Guess for Thrust Controls
yaw = 0;                                  % Initial Yaw
State_vector = zeros(8,1);                % Drone States
p_State_vector = zeros(8,1);              % Perturbed States
State_horizon = zeros(8,N);               % Drone State Horizon
torques_history = zeros(3,time/dt-N-1);   % History of Torques Applied
angles_history = zeros(3,time/dt-N-1);    % History of Angles
thrust_history = zeros(1,time/dt-N-1);    % History of Thrusts Applied
angles_ref_history = zeros(3,time/dt-N-1);% History of Angle References
instant_cost_pos = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_pos = zeros(1, length(T)); % Integrated cost
instant_cost_att = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_att = zeros(1, length(T)); % Integrated cost
% Reference Selection
Line_reference=0;                         % Set One Desired Reference Equals to 1 
Step_reference=1;
Sin_reference=0;
% [reference_x,reference_y,reference_z] = Reference_Generation(Line_reference,Step_reference,Sin_reference,time,dt);
% reference = [reference_x; reference_y; reference_z];
reference=r_smooth;
% Simulate

%% Test 2: disturbance and uncertainty
add_wind = 0;
add_sensor_noise = 0.06;
add_actuator_noise = 1;
add_saturation = 1; % logical value
load("wind_simulation.mat")
L = 0.2;
drag = 0.02; % (drag coef)
max_Ft = 2*m*grav;
Tmin = 0;
Tmax = max_Ft/4;
Max_torque = Tmax*L*2;

sensor_uncert = [0.02, 0.02, 0.05, 0.05, 0.05, 0.05, 0.15, 0.15, 0.5, 0.005, 0.005, 0.005]';
actuator_uncert = [0.1*max_Ft, 0.01*Max_torque, 0.01*Max_torque, 0.02*Max_torque]';

u1_max = Max_torque; u2_max = Max_torque; u3_max = 0.7*Max_torque;
Umax = [u1_max; u2_max; u3_max];
tic
for t=1:time/dt-N-1
   
    % Print the Current Time in the Simulation
    if mod(t, 1/dt) == 0
        fprintf('time = %d sec\n',t*dt)                 %
    end

    % [x;y;z;u;v;w;pitch;roll;thrust;pitch_cmd;roll_cmd]
    current_reference(1:3,:) = reference(:, t+1: t+N);
    current_reference(4:6,:)= (reference(:,t+2:t+N+1)-reference(:, t+1: t+N))/dt;
    current_reference(7:8,:)=0;
    current_reference(9,:)=9.81/cos(X(7))/cos(X(8));
    
    % Run MPC to find the Optimal Position Controls
    [pos_controls,J_current]=MPC_Position_Controller_noNN(current_reference,X,pos_controls,N,J,JN,dt,K,angular_time_constants,decay_rate,alpha,epsilon);
    
    % Display cost of the prediction horizon
    if mod(t, 1/dt) == 0
        fprintf('cost is %f\n',J_current)
    end
    
    pitch = pos_controls(2);
    roll = pos_controls(3);
    yaw = 1;
    ax = -(pos_controls(1)/m)*(sin(pitch)*sin(yaw)+ cos(pitch)*cos(yaw)*sin(roll));
    ay = -(pos_controls(1)/m)*(cos(pitch)*sin(yaw)*sin(roll) - cos(yaw)*sin(pitch));
    az = grav - (pos_controls(1)/m)*(cos(pitch)*cos(roll));
    
    % Attitude Controller Operates at 10 times the frequency as position controller
    for i=1:10 
        ft = pos_controls(1);
        full_ref(:,m) = ref_for_cost(:,t);
        U_pos(:,m) = [ax;ay;az];
        angles_ref(:,t) = [pos_controls(2:3,1); 1];                  % Desired angles: [pitch, roll, yaw]
        % Run the Attitude Controller
        [torques,angle_error_old,integral_error]=Attitude_Controller(angles_ref(:,t),X(7:9),angle_error_old,m,angles_history,angles_ref_history,integral_error,Kp,Kd,Ki,dt);

        % Store Thrust Applied
        %% saturate the torques to the max torque the drone can produce
        % if add_saturation ==1
        %     for p = 1:3
        %         if torques(p) > Max_torque
        %             torques(p) = Max_torque;
        %         elseif torques(p) < -Max_torque
        %             torques(p) = -Max_torque;
        %         end
        %     end
        %     % solve for the thrust of each motor given those torques    
        %     T1 = (ft - torques(1)/(2*L) - torques(2)/(2*L) + torques(3)/(4*drag)) / 4;
        %     T2 = (ft - torques(1)/(2*L) + torques(2)/(2*L) + torques(3)/(4*drag)) / 4;
        %     T3 = (ft + torques(1)/(2*L) - torques(2)/(2*L) - torques(3)/(4*drag)) / 4;
        %     T4 = (ft + torques(1)/(2*L) + torques(2)/(2*L) - torques(3)/(4*drag)) / 4;
        %     each_motor_thrust = [T1;T2;T3;T4];
        % 
        %     % saturate each motor thrust to its bounded values
        %     for j = 1:4
        %         if each_motor_thrust(j) > Tmax
        %             each_motor_thrust(j) = Tmax;
        %         elseif each_motor_thrust(j) < Tmin
        %             each_motor_thrust(j) = Tmin;
        %         end
        %     end
        % 
        %     if ft>max_Ft
        %         ft = max_Ft;
        %     elseif ft<0
        %         ft = 0;
        %     end
        % 
        % end
        U(1,:) = ft;                             % Thrust controls from MPC
        U(2:4,1) = torques;
        % %% Case 2 and 3 Noise
        % X = add_noise(X, sensor_uncert, add_sensor_noise); % add noise
        % 
        % %% Case 4 wind noise
        % if add_wind == 1
        %     [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,m), X, rho, Cd, A, r, mass, Ix, Iy, Iz);
        %     X = X + 0.004*[0;0;0;
        %                axyz_uncert;
        %                0;0;0;
        %                alphaxyz_uncert];
        % end
        
        X = DroneDynamicsFunction(X,U(:,1),1,.3,.4,.5,dt/10);   % Propagate through time
        X_values(:,m)=X;                            % Store States
        U(:,1) = add_noise(U(:,1), actuator_uncert, add_actuator_noise);
        U_values(:,m)=U(:,1);                       % Store Controls
        
        % Record Values
        torques_history(:, m) = torques;        % Store Torques Applied
        angles_ref_history(:, m) = angles_ref(:,t);  % Store Reference Angles
        angles_history(:, m) = X(7:9);          % Store Actual Angles 
        thrust_history(:,m) = U(1,1);   
        
        pos_error(:,m) = X(1:6) - full_ref(:,m);
        Att_error(:,m) = X(7:12) - [angles_ref(:,t); deriv(angles_ref,t,dt)];
        instant_cost_pos(m) = pos_error(:,m)' * Q_pos * pos_error(:,m) + U_pos(:,m)' * R_pos * U_pos(:,m);
        instant_cost_att(m) = Att_error(:,m)' * Q_att * Att_error(:,m) + torques' * R_att * torques;
        
       if m > 1
            cumulative_cost_pos(m) = cumulative_cost_pos(m-1) + instant_cost_pos(m);
            cumulative_cost_att(m) = cumulative_cost_att(m-1) + instant_cost_att(m);
       end
       
       m=m+1; 
    end
end
MPC_final_time = toc;

MPC_results.X = X_values(:,1:12500);
MPC_results.U_pos = U_pos(:,1:12500);
MPC_results.U = U_values(:,1:12500);
MPC_results.error = [pos_error(:,1:12500);Att_error(:,1:12500)];
MPC_results.r_initial = full_ref(:,1:12500);
MPC_results.inscost = [instant_cost_pos(:,1:12500);instant_cost_att(:,1:12500)];
MPC_results.cumcost = [cumulative_cost_pos(:,1:12500);cumulative_cost_att(:,1:12500)];
MPC_results.simtime = MPC_final_time;

if (add_sensor_noise > 0 || add_actuator_noise > 0) && add_saturation ~=1
    save("MPC_compare_workspace_noise.mat","MPC_results")
    disp("Saving noise only simulation")
elseif add_saturation == 1
    save("MPC_compare_workspace_noise_saturation.mat","MPC_results")
    disp("Saving noise and saturation simulation")
elseif add_wind == 1
    save("MPC_compare_workspace_wind.mat","MPC_results")
    disp("Saving wind simulation")
else 
    save("MPC_compare_workspace.mat","MPC_results")
    disp("Saving ideal simulation")
end
% save("MPC_wind_case1-1.mat","MPC_results")


%% Plotting
% Position History
X_values_pos = X_values(1:3, 1:time/dt-N-1);
reference_pos = reference(:, 1:time/dt-N-1);


% % Plot Position
% figure;
% subplot(3, 1, 1);
% plot((1:time/dt-N-1) * dt, X_values_pos(1, :), 'b', (1:time/dt-N-1) * dt, reference_pos(1, :), 'r--');
% title('X Position');
% xlabel('Time (seconds)');
% ylabel('X Position(m)');
% legend('X values', 'Reference');
% subplot(3, 1, 2);
% plot((1:time/dt-N-1) * dt, X_values_pos(2, :), 'b', (1:time/dt-N-1) * dt, reference_pos(2, :), 'r--');
% title('Y Position');
% xlabel('Time (seconds)');
% ylabel('Y Position(m)');
% legend('Y values', 'Reference');
% subplot(3, 1, 3);
% plot((1:time/dt-N-1) * dt, -X_values_pos(3, :), 'b', (1:time/dt-N-1) * dt, -reference_pos(3, :), 'r--');
% title('Z Position');
% xlabel('Time (seconds)');
% ylabel('Z Position(m)');
% legend('Z values', 'Reference');

% 3D Position Plot
figure;
plot3(full_ref(1, :), full_ref(2, :), -full_ref(3, :), 'r--', 'LineWidth', 2); % Reference trajectory
hold on;
plot3(X_values(1, :), X_values(2, :), -X_values(3, :), 'b', 'LineWidth', 2); % Drone trajectory
hold off;
title('3D Trajectory: Drone Tracking vs Reference');
xlabel('X Position(m)');
ylabel('Y Position(m)');
zlabel('Z Position(m)');
legend('Reference Trajectory', 'Drone Trajectory');
grid on;
axis equal;

% Pitch: Target vs Actual Angles
figure;
subplot(3, 1, 1);
plot((1:length(angles_ref_history(1, :))) * dt/10, angles_ref_history(1, :), 'g--', 'LineWidth', 2); hold on;
plot((1:length(angles_history(1, :))) * dt/10, angles_history(1, :), 'r:', 'LineWidth', 2);
title('Pitch Angle: Target vs Actual');
xlabel('Time (seconds)');
ylabel('Angle (radians)');
legend('Pitch Ref Angle', 'Pitch Actual Angle');
grid on;
hold off;

% Roll: Target vs Actual Angles
subplot(3, 1, 2);
plot((1:length(angles_ref_history(2, :))) * dt/10, angles_ref_history(2, :), 'g--', 'LineWidth', 2); hold on;
plot((1:length(angles_history(2, :))) * dt/10, angles_history(2, :), 'r:', 'LineWidth', 2);
title('Roll Angle: Target vs Actual');
xlabel('Time (seconds)');
ylabel('Angle (radians)');
legend('Roll Ref Angle', 'Roll Actual Angle');
grid on;
hold off;

% Yaw: Target vs Actual Angles
subplot(3, 1, 3);
plot((1:length(angles_ref_history(3, :))) * dt/10, angles_ref_history(3, :), 'g--', 'LineWidth', 2); hold on;
plot((1:length(angles_history(3, :))) * dt/10, angles_history(3, :), 'r:', 'LineWidth', 2);
title('Yaw Angle: Target vs Actual');
xlabel('Time (seconds)');
ylabel('Angle (radians)');
legend('Yaw Ref Angle', 'Yaw Actual Angle');
grid on;
hold off;

% Torques and Thrust
figure;
subplot(4, 1, 1);
plot((1:length(torques_history(1, :))) * dt/10, torques_history(1, :), 'r-', 'LineWidth', 2);
title('X Torque over Time');
xlabel('Time (seconds)');
ylabel('Torque (Nm)');
legend('X Torque');
grid on;
subplot(4, 1, 2);
plot((1:length(torques_history(2, :))) * dt/10, torques_history(2, :), 'g-', 'LineWidth', 2);
title('Y Torque over Time');
xlabel('Time (seconds)');
ylabel('Torque (Nm)');
legend('Y Torque');
grid on;

subplot(4, 1, 3);
plot((1:length(torques_history(3, :))) * dt/10, torques_history(3, :), 'b-', 'LineWidth', 2);
title('Z Torque over Time');
xlabel('Time (seconds)');
ylabel('Torque (Nm)');
legend('Z Torque');
grid on;
subplot(4, 1, 4);
plot((1:length(thrust_history(1, :))) * dt/10, thrust_history(1, :), 'y-', 'LineWidth', 2);
title('Thrust over Time');
xlabel('Time (seconds)');
ylabel('Thrust (N)');
legend('Thrust');
grid on;

function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end

function pqr = deriv(angles, t, dt)
if t == 1
    pqr = ((angles(:, t) - 0)/dt);
elseif t > 1
    pqr = ((angles(:, t) - angles(:, t - 1))/(dt));
end
% Limit pqr to be between -5 and 5
pqr(pqr > 1) = 1;
pqr(pqr < -1) = -1;
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
    V_rel = X(4:6) - Wxyz_body;
    F_wind = -0.5 * rho * Cd .*diag(A) * (V_rel .* abs(V_rel)); % Compute wind forces
    
    F_wind_motors = repmat(F_wind, 1, 4) + 0.01*randn(3,4);

    M_wind_motors = cross(r, F_wind_motors, 1);
    M_wind = sum(M_wind_motors, 2);  
    
    accn_dist = F_wind/m;
    alpha_dist=    [M_wind(1)/Ix;
                    M_wind(2)/Iy;
                    M_wind(3)/Iz];
end

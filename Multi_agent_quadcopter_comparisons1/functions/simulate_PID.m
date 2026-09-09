function pid_results = simulate_PID(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
rng(seed);
R_pos = position.R_pos;
Q_pos = position.Q_pos;
R_att = attitude.R_att;
Q_att = attitude.Q_att;
dt      = global_parameters.dt;
g       = global_parameters.g;
tf      = global_parameters.tf;
m       = global_parameters.m;
Ix      = global_parameters.Ix;
Iy      = global_parameters.Iy;
Iz      = global_parameters.Iz;
W_nominal = global_parameters.W_nominal;
state_noise = sim_params.state_noise;
control_noise =sim_params.control_noise;
saturation_logic = sim_params.saturation_logic;
wind_logic = sim_params.wind_logic; 
reference_smoothing_logic = sim_params.reference_smoothing_logic;

load("z_PID_Gains.mat")
% load("z_PID_Gains_TVT_paper_with_hold.mat")

T = 0:dt:tf-dt;
N = length(T);

% Preallocating variables
X               = zeros(12,N);
U               = zeros(4, length(T)); % Control input trajectory
X_ref           = zeros(12, length(T)); % reference trajectory
r_yaw           = global_parameters.r_yaw; 
torques         = zeros(3,N);   % output of attitude SNAC
att_error       = zeros(6,N);   % error 
pos_error       = zeros(6,N);
r_initial       = zeros(3,N-1); % original trajectory
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


X(:,1) = IC; % defining inital x as the initial condtion (IC)

% determining modified reference trajectory based on original trajectory
for i = 1:length(T)
    r_initial(:,i) = ref(T(i));
end

% new logic for parameterized reference smoothing
if reference_smoothing_logic == 1 
    smooth_r_position = smooth(r_initial, X(1:3), X(4:6), T);
else
    smooth_r_position = r_initial;
end
r_smooth = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];
%% eso gains
eso_state.p_hat = IC(1:3);
eso_state.v_hat = IC(4:6);
eso_state.d_hat = zeros(3,1);
use_ESO = 0;
%% position control loop stuff
desired_vel_hold   = zeros(3,1);
desired_accel_hold = [0;0;g];
dt_pos = 10 * dt;

tic
for i = 1:length(T)
    %% add sensor noise
    if state_noise ~= 0
        X(:,i) = add_state_noise(X(:,i), state_noise);
    end
    %% add wind disturbence
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), m, Ix, Iy, Iz);
        X(:,i) = X(:,i)+ dt*[0;0;0;
                           axyz_uncert;
                           0;0;0;
                           alphaxyz_uncert];
    end
% if use hold in position loop
%% **1. Compute Position Errors**
    X_ref(1:6, i) = r_smooth(:,i);  % Desired position & velocity
    pos_error(1:3,i) = X_ref(1:3, i) - X(1:3, i);  % Position error

    if mod(i-1,10) == 0
        % remove the look ahead
        % idx = min(i + 10, N);
        r_future = r_smooth(:, i);

        pos_error_future = r_future(1:3) - X(1:3,i);

        %% Position PID
        integral_pos = integral_pos + pos_error_future * dt_pos;
        derivative_pos = (pos_error_future - previous_error_pos) / dt_pos;
        previous_error_pos = pos_error_future;

        desired_vel_hold = [
            PID_Gains.x.Kp * pos_error_future(1) + PID_Gains.x.Ki * integral_pos(1) + PID_Gains.x.Kd*(PID_Gains.x.N/(1+PID_Gains.x.N))*derivative_pos(1);
            PID_Gains.y.Kp * pos_error_future(2) + PID_Gains.y.Ki * integral_pos(2) + PID_Gains.y.Kd*(PID_Gains.y.N/(1+PID_Gains.y.N))*derivative_pos(2);
            PID_Gains.z.Kp * pos_error_future(3) + PID_Gains.z.Ki * integral_pos(3) + PID_Gains.z.Kd*(PID_Gains.z.N/(1+PID_Gains.z.N))*derivative_pos(3);
        ];

        %% Velocity PID
        vel_error_hold = desired_vel_hold - X(4:6,i);

        integral_vel = integral_vel + vel_error_hold * dt_pos;
        derivative_vel = (vel_error_hold - previous_error_vel) / dt_pos;
        previous_error_vel = vel_error_hold;

        desired_accel_hold = [
            PID_Gains.u.Kp * vel_error_hold(1) + PID_Gains.u.Ki * integral_vel(1) + PID_Gains.u.Kd*(PID_Gains.u.N/(1+PID_Gains.u.N))*derivative_vel(1);
            PID_Gains.v.Kp * vel_error_hold(2) + PID_Gains.v.Ki * integral_vel(2) + PID_Gains.v.Kd*(PID_Gains.v.N/(1+PID_Gains.v.N))*derivative_vel(2);
            PID_Gains.w.Kp * vel_error_hold(3) + PID_Gains.w.Ki * integral_vel(3) + PID_Gains.w.Kd*(PID_Gains.w.N/(1+PID_Gains.w.N))*derivative_vel(3);
        ] + [0;0;g];

            %% ESO
        if use_ESO == 1 && wind_logic == 1
            w0 = 2;
            if i == 1
                U_prev = [m*g;0;0;0];  % hover guess
            else
                U_prev = U(:,i-1);     % applied control from previous step
            end
    
            [eso_state, d_hat_pos] = ESO_position(eso_state, X(:,i), U_prev, m, 10*dt, w0);
            desired_accel_hold = desired_accel_hold - d_hat_pos;
        end
        desired_vel = desired_vel_hold;
        desired_accel = desired_accel_hold;
        [ft, r_pitch, r_roll] = system_solve(-desired_accel(1), -desired_accel(2), -(desired_accel(3)), r_yaw(i), m);
        angles = [r_pitch; r_roll; r_yaw(i)];
    else
        desired_vel = desired_vel_hold;
        desired_accel = desired_accel_hold;
    end


    vel_error(:,i) = desired_vel - X(4:6,i);
    pos_error(4:6,i) = vel_error(:,i);
%% end of hold positon loop
%% normal rate positon loop (do not delete)
 % %% **1. Compute Position Errors**
 %    X_ref(1:6, i) = r_smooth(:,i);  % Desired position & velocity
 %    pos_error(1:3,i) = X_ref(1:3, i) - X(1:3, i);  % Position error
 %    % can use reference or velocity PID to get vel error
 %    % vel_error(:,i) = X_ref(4:6, i) - X(4:6, i);  % Velocity error
 % 
 %    %% **2. Position PID (outputs desired velocities u, v, w)**
 %    integral_pos = integral_pos + pos_error(1:3,i) * dt;
 %    derivative_pos = (pos_error(1:3,i) - previous_error_pos) / dt;
 %    previous_error_pos = pos_error(1:3,i);
 % 
 %    desired_vel = [
 %        PID_Gains.x.Kp * pos_error(1,i) + PID_Gains.x.Ki * integral_pos(1) + PID_Gains.x.Kd*(PID_Gains.x.N / (1+PID_Gains.x.N))* derivative_pos(1);
 %        PID_Gains.y.Kp * pos_error(2,i) + PID_Gains.y.Ki * integral_pos(2) + PID_Gains.y.Kd*(PID_Gains.y.N / (1+PID_Gains.y.N))* derivative_pos(2);
 %        PID_Gains.z.Kp * pos_error(3,i) + PID_Gains.z.Ki * integral_pos(3) + PID_Gains.z.Kd*(PID_Gains.z.N / (1+PID_Gains.z.N))*derivative_pos(3);
 %    ];
 % 
 %    vel_error(:,i) = desired_vel - X(4:6, i);
 %    pos_error(4:6,i) = vel_error(:,i);
 %    %% **3. Velocity PID (outputs desired accelerations a_x, a_y, a_z)**
 %    integral_vel = integral_vel + vel_error(:,i) * dt;
 %    derivative_vel = (vel_error(:,i) - previous_error_vel) / dt;
 %    previous_error_vel = vel_error(:,i);
 % 
 %    desired_accel = [
 %        PID_Gains.u.Kp * vel_error(1,i) + PID_Gains.u.Ki * integral_vel(1) + PID_Gains.u.Kd *(PID_Gains.u.N / (1+PID_Gains.u.N))* derivative_vel(1);
 %        PID_Gains.v.Kp * vel_error(2,i) + PID_Gains.v.Ki * integral_vel(2) + PID_Gains.v.Kd *(PID_Gains.v.N / (1+PID_Gains.v.N))* derivative_vel(2);
 %        PID_Gains.w.Kp * vel_error(3,i) + PID_Gains.w.Ki * integral_vel(3) + PID_Gains.w.Kd *(PID_Gains.w.N / (1+PID_Gains.w.N))* derivative_vel(3);
 %    ] + [0; 0; g];  % Gravity compensation
%% end of normal rate positon loop


    %% Convert acceleration to thrust & desired angles
    % [ft, r_pitch, r_roll] = system_solve(-desired_accel(1), -desired_accel(2), -(desired_accel(3)), X(9,i), m);
    % angles = [r_pitch; r_roll; r_yaw(i)];

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
    integral_ang_unsat = integral_ang + ang_vel_error * dt;
    derivative_ang = (ang_vel_error - previous_error_ang) / dt;
    previous_error_ang = ang_vel_error;

    torques(:,i) = [
        PID_Gains.p.Kp * ang_vel_error(1) + PID_Gains.p.Ki * integral_ang_unsat(1) + PID_Gains.p.Kd *(PID_Gains.p.N / (1+PID_Gains.p.N))* derivative_ang(1);
        PID_Gains.q.Kp * ang_vel_error(2) + PID_Gains.q.Ki * integral_ang_unsat(2) + PID_Gains.q.Kd *(PID_Gains.q.N / (1+PID_Gains.q.N))* derivative_ang(2);
        PID_Gains.r.Kp * ang_vel_error(3) + PID_Gains.r.Ki * integral_ang_unsat(3) + PID_Gains.r.Kd *(PID_Gains.r.N / (1+PID_Gains.r.N))* derivative_ang(3);
    ];

    %% ANTI-windup /auxiliary Dynamics compensation
    if saturation_logic == 1
        U_cmd = [ft; torques(:,i)];
        U_sat = saturate_controls(U_cmd);
      
        DeltaU = U_sat - U_cmd;
        Kaw_tau = 1;  % tune 0.1 to 5
        integral_ang = integral_ang + dt*( ang_vel_error + Kaw_tau * DeltaU(2:4) );

        U(:,i) = U_sat;
    else
        integral_ang = integral_ang + ang_vel_error * dt;
        U(:,i) = [ft; torques(:,i)];
    end


    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end
    %% Update Nonlinear Dynamics using Euler integration
    f_x = Full_f(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g(X(:, i), m,Ix,Iy,Iz);
    
    %% State update using Euler integration
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end

    %% remember this is comtinous cost to discrete cost ADD dt term
    Att_error(:,i) = [att_error;ang_vel_error];
end

pid_sim_time = toc;
%% save all useful info in a struck
pid_results.X = X;
pid_results.U = U;
pid_results.X_ref = X_ref;
pid_results.error = [pos_error;Att_error];
pid_results.simtime = pid_sim_time;
pid_error = pid_results.error;
pid_tracking_error = (pid_error(1,:).^2 + pid_error(2,:).^2 + pid_error(3,:).^2).^0.5;
pid_results.pid_tracking_error = pid_tracking_error;
pid_results.pid_tracking_error_norm = dt*sum(pid_tracking_error,2);


% [tot_cost, track_cost, control_cost] = calculate_cost(pid_results.error,pid_results.U,dt);
% pid_results.all_cost = [tot_cost, track_cost, control_cost];


function pid_results = simulate_PID(position, attitude, global_parameters, ref, IC, sim_params)
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
payload_uncert_full = sim_params.payload_uncert;
payload_uncert_logic = payload_uncert_full(1);
payload_uncert = payload_uncert_full(2:5);
inertia_params = [m; Ix; Iy; Iz];

load("z_PID_Gains.mat")

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
instant_cost_pos = zeros(1, N); % Instantaneous cost
cumulative_cost_pos = zeros(1, N); % Integrated cost
instant_cost_att = zeros(1, N); % Instantaneous cost
cumulative_cost_att = zeros(1, N); % Integrated cost
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

%% wind sim stuff
Cd = [.8; 0.8;1.2]; % Drag coefficient
L = 0.2; % Half arm length (m)
h = 0.05;
A = [L*2*h, L*2*h, 2*L*L]; % Approximate frontal areas [x, y, z] in m^2
r = [ L,  L, -L, -L;   % X positions of motors
      L, -L, -L,  L;   % Y positions of motors
      0,  0,  0,  0 ]; % Z positions (motors at same height)
rho = 1.225; % Air density (kg/m^3)
Wxyz = gen_wind_vector(W_nominal,dt,tf);

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

% --- Saturation compensation (anti-windup / auxiliary dynamics) ---
% xi_aw = zeros(4,1);          % auxiliary state for [ft; taux; tauy; tauz]
% K_aw_tau  = 5*diag([1.0 1.0 .5]);  % only torque channels by default
% k_xi  = 90;
% ESO GAINS
eso_state.p_hat = IC(1:3);
eso_state.v_hat = IC(4:6);
eso_state.d_hat = zeros(3,1);
% ESO GAINS
% ieso_state.p_hat = IC(1:3);
% ieso_state.v_hat = IC(4:6);
% ieso_state.d_hat = zeros(3,1);
use_ESO = 1;

tic
for i = 1:length(T)
    %% add payload uncertianty 10 times during simulation
    if payload_uncert_logic == 1 && mod(i,500) == 0
        m = inertia_params(1) + payload_uncert(1)*randn;
        Ix = inertia_params(2) + payload_uncert(2)*randn;
        Iy = inertia_params(3) + payload_uncert(3)*randn;
        Iz = inertia_params(4) + payload_uncert(4)*randn;
    end
    %% add sensor noise
    if state_noise ~= 0
        X(:,i) = add_state_noise(X(:,i), state_noise);
    end
    %% add wind disturbence
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), rho, Cd, A, r, m, Ix, Iy, Iz);
        X(:,i) = X(:,i)+ dt*[0;0;0;
                           axyz_uncert;
                           0;0;0;
                           alphaxyz_uncert];
    end

    %% **1. Compute Position Errors**
    X_ref(1:6, i) = r_smooth(:,i);  % Desired position & velocity
    pos_error(1:3,i) = X_ref(1:3, i) - X(1:3, i);  % Position error
    % can use reference or velocity PID to get vel error
    % vel_error(:,i) = X_ref(4:6, i) - X(4:6, i);  % Velocity error

    %% **2. Position PID (outputs desired velocities u, v, w)**
    integral_pos = integral_pos + pos_error(1:3,i) * dt;
    derivative_pos = (pos_error(1:3,i) - previous_error_pos) / dt;
    previous_error_pos = pos_error(1:3,i);
    
    desired_vel = [
        PID_Gains.x.Kp * pos_error(1,i) + PID_Gains.x.Ki * integral_pos(1) + PID_Gains.x.Kd*(PID_Gains.x.N / (1+PID_Gains.x.N))* derivative_pos(1);
        PID_Gains.y.Kp * pos_error(2,i) + PID_Gains.y.Ki * integral_pos(2) + PID_Gains.y.Kd*(PID_Gains.y.N / (1+PID_Gains.y.N))* derivative_pos(2);
        PID_Gains.z.Kp * pos_error(3,i) + PID_Gains.z.Ki * integral_pos(3) + PID_Gains.z.Kd*(PID_Gains.z.N / (1+PID_Gains.z.N))*derivative_pos(3);
    ];

    vel_error(:,i) = desired_vel - X(4:6, i);
    pos_error(4:6,i) = vel_error(:,i);
    %% **3. Velocity PID (outputs desired accelerations a_x, a_y, a_z)**
    integral_vel = integral_vel + vel_error(:,i) * dt;
    derivative_vel = (vel_error(:,i) - previous_error_vel) / dt;
    previous_error_vel = vel_error(:,i);

    desired_accel = [
        PID_Gains.u.Kp * vel_error(1,i) + PID_Gains.u.Ki * integral_vel(1) + PID_Gains.u.Kd *(PID_Gains.u.N / (1+PID_Gains.u.N))* derivative_vel(1);
        PID_Gains.v.Kp * vel_error(2,i) + PID_Gains.v.Ki * integral_vel(2) + PID_Gains.v.Kd *(PID_Gains.v.N / (1+PID_Gains.v.N))* derivative_vel(2);
        PID_Gains.w.Kp * vel_error(3,i) + PID_Gains.w.Ki * integral_vel(3) + PID_Gains.w.Kd *(PID_Gains.w.N / (1+PID_Gains.w.N))* derivative_vel(3);
    ] + [0; 0; g];  % Gravity compensation

    %% ESO
    if use_ESO == 1 && wind_logic == 1
        w0 = 30;
        if i == 1
            U_prev = [m*g;0;0;0];  % hover guess
        else
            U_prev = U(:,i-1);     % applied control from previous step
        end

        [eso_state, d_hat_pos] = ESO_position(eso_state, X(:,i), U_prev, m, dt, w0);
        desired_accel = desired_accel - d_hat_pos;
    end
    %% position Improved ESO from paper 
    % if use_ESO == 1 && wind_logic == 1
    %     w0 = 100;  % try 5–15
    %     k_pos =1*[1;1; 1];
    %     [ieso_state, d_hat_force, e_obs] = IESO_position_update_NED(ieso_state, X(:,i), desired_accel, m, dt, w0);
    % 
    %     % Improved compensation term: (d_hat + k*m*e_obs)/m
    %     desired_accel = desired_accel - (d_hat_force + (k_pos.*m).*e_obs)/m;
    % end
    %% Convert acceleration to thrust & desired angles
    [ft, r_pitch, r_roll] = system_solve(-desired_accel(1), -desired_accel(2), -(desired_accel(3)), X(9,i), m);
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
    integral_ang_unsat = integral_ang + ang_vel_error * dt;
    derivative_ang = (ang_vel_error - previous_error_ang) / dt;
    previous_error_ang = ang_vel_error;

    torques(:,i) = [
        PID_Gains.p.Kp * ang_vel_error(1) + PID_Gains.p.Ki * integral_ang_unsat(1) + PID_Gains.p.Kd *(PID_Gains.p.N / (1+PID_Gains.p.N))* derivative_ang(1);
        PID_Gains.q.Kp * ang_vel_error(2) + PID_Gains.q.Ki * integral_ang_unsat(2) + PID_Gains.q.Kd *(PID_Gains.q.N / (1+PID_Gains.q.N))* derivative_ang(2);
        PID_Gains.r.Kp * ang_vel_error(3) + PID_Gains.r.Ki * integral_ang_unsat(3) + PID_Gains.r.Kd *(PID_Gains.r.N / (1+PID_Gains.r.N))* derivative_ang(3);
    ];
    %% **6. Apply Control Inputs to Drone Model**
    % U(:,i) = [ft; torques(:,i)];
    % 
    % if saturation_logic == 1
    %     U(:,i) = saturate_controls(U(:,i));
    % end

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
    Att_error(:,i) = [att_error;desired_angular_rates];
    instant_cost_pos(i) = dt * (pos_error(:,i)' * Q_pos * pos_error(:,i) + desired_accel' * R_pos * desired_accel);
    instant_cost_att(i) = dt * (Att_error(:,i)' * Q_att * Att_error(:,i) + torques(:,i)' * R_att * torques(:,i));
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
   end
end

pid_sim_time = toc;
%% save all useful info in a struck
pid_results.X = X;
pid_results.U = U;
pid_results.X_ref = X_ref;
pid_results.error = [pos_error;Att_error];
pid_results.inscost = [instant_cost_pos; instant_cost_att];
pid_results.cumcost = [cumulative_cost_pos; cumulative_cost_att];
pid_c_cost_total = sum(pid_results.cumcost, 1);
pid_results.pid_c_cost_total = pid_c_cost_total;
pid_results.simtime = pid_sim_time;
pid_error = pid_results.error;
pid_tracking_error = (pid_error(1,:).^2 + pid_error(2,:).^2 + pid_error(3,:).^2).^0.5;
pid_results.pid_tracking_error = pid_tracking_error;
pid_results.pid_tracking_error_norm = dt*sum(pid_tracking_error,2);

[tot_cost, track_cost, control_cost] = calculate_cost(pid_results.error,pid_results.U,dt);
pid_results.all_cost = [tot_cost, track_cost, control_cost];

function Backstepping_results = simulate_Backstepping(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
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

T = 0:dt:tf-dt;
N = length(T);
% Preallocating variables
X               = zeros(12,N);
U               = zeros(4, length(T));
X_ref           = zeros(12, length(T));
angles          = zeros(3,length(T));
r_yaw           = global_parameters.r_yaw;
U_pos            = zeros(3,N);     
torques         = zeros(3,N);
att_error       = zeros(6,N);
pos_error       = zeros(6,N);
r_initial       = zeros(3,N-1);

X(:,1) = IC;

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

r_xyz_ddot = discrete_deriv(r_smooth(4:6,:),dt);

%% Gains (might need tuning)
Kp_pos = diag([1 1 1]);
Kv_pos = .7*diag([3.5 3.5 4.5]);
Kc_pos = eye(3);
Keta   = diag([6.0 6.0 4.0]);
Komega = diag([2.5 2.5 1.8]);

omega_d_prev = zeros(3,1);

Trans = @(angles) [ 1,  sin(angles(1))*tan(angles(2)),  cos(angles(1))*tan(angles(2));
                    0,  cos(angles(1)),            -sin(angles(1));
                    0,  sin(angles(1))/cos(angles(2)),  cos(angles(1))/cos(angles(2))];
ohm_d = zeros(3,N);
omega_d_dot_filt = zeros(3,1);

% --- Saturation compensation (anti-windup / auxiliary dynamics) ---
xi_aw = zeros(4,1);          % auxiliary state for [ft; taux; tauy; tauz]
K_aw_tau  = 1*diag([1.0 1.0 .5]);  % only torque channels by default
k_xi  = 50;

tic

for i = 1:N
    %% add sensor noise
    if state_noise ~= 0
        X(:,i) = add_state_noise(X(:,i), state_noise);
    end
    %% add wind disturbence
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i),m, Ix, Iy, Iz);
        X(:,i) = X(:,i)+ dt*[0;0;0;
                           axyz_uncert;
                           0;0;0;
                           alphaxyz_uncert];
    end 
    %% reference at this timestep
    X_ref(1:6, i) = r_smooth(:,i);
    %% 6x1 position error
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);

    if mod(i-1,10) == 0
        % remove the look ahead
        % idx = min(i + 10, N);
        r_future = r_smooth(:, i);
        r_xzy_ddot_future = r_xyz_ddot(:,i);

        e_p = X(1:3,i) - r_future(1:3);
        v_d = r_future(4:6) - Kp_pos*e_p;
        e_v = X(4:6,i) - v_d;
        v_d_dot = r_xzy_ddot_future - Kp_pos*(X(4:6,i) - r_future(4:6));
        a_cmd = v_d_dot - Kv_pos*e_v - Kc_pos*e_p;
        U_pos(:,i) = [a_cmd(1); a_cmd(2); a_cmd(3)];
        [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i)-g, r_yaw(i), m);
        angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
        X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];
    else
        U_pos(:,i) = U_pos(:,i-1);
        X_ref(7:12,i) = X_ref(7:12,i-1);
        angles(:,i) = angles(:,i-1);
    end


    % [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i)-g, r_yaw(i), m);
    % angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    % X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    
    %% INNER LOOP BACKSTEPPING-LIKE (attitude)
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    e_eta = att_error(1:3,i);

    Tmat = Trans(X(7:9,i));

    eta_d_dot = X_ref(10:12,i);
    eta_dot_cmd = eta_d_dot - Keta*e_eta;
    ohm_d(:,i) = Tmat \ eta_dot_cmd;

    %% first order filter for omega_d_dot
    if i == 1
        omega_d_dot_raw = zeros(3,1);
    else
        omega_d_dot_raw = (ohm_d(:,i) - omega_d_prev)/dt;
    end
    omega_d_prev = ohm_d(:,i);

    alpha = 0.9;  % filter factor (0.8-0.98)
    omega_d_dot = alpha * omega_d_dot_filt + (1-alpha) * omega_d_dot_raw;
    omega_d_dot_filt = omega_d_dot;

    omg = X(10:12,i);
    e_omega = omg - ohm_d(:,i);
    coupling = [ (Iy - Iz)*omg(2)*omg(3);
                 (Iz - Ix)*omg(1)*omg(3);
                 (Ix - Iy)*omg(1)*omg(2)];

    I = diag([Ix Iy Iz]);
    

    %% ANTI-windup /auxiliary Dynamics compensation
    if saturation_logic == 1
        torques(:,i) = coupling + I*(omega_d_dot - Komega*e_omega - K_aw_tau*xi_aw(2:4));
        U_cmd = [ft; torques(:,i)];
        % Saturate
        U_sat = saturate_controls(U_cmd);

        % Saturation mismatch
        DeltaU = U_sat - U_cmd;

        xi_aw = xi_aw + dt * (-k_xi * xi_aw + DeltaU);

        U(:,i) = U_sat;
    else
        torques(:,i) = coupling + I*(omega_d_dot - Komega*e_omega);
        U(:,i) = [ft; torques(:,i)];
    end


    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end

    %% dynamics update
    f_x = Full_f(X(:,i), g, Ix, Iy, Iz);
    g_x = Full_g(X(:,i), m, Ix, Iy, Iz);

    if i < N
        X(:,i+1) = X(:,i) + dt*(f_x + g_x*U(:,i));
    end
end

Backstepping_sim_time = toc;

%% outputs 
%% save all useful info in a struck
Backstepping_results.X = X;
Backstepping_results.U = U;
Backstepping_results.X_ref = X_ref;
Backstepping_results.U_pos = U_pos;
Backstepping_results.error = [pos_error;att_error];
Backstepping_results.simtime = Backstepping_sim_time;
Backstepping_error = Backstepping_results.error;
Backstepping_tracking_error = (Backstepping_error(1,:).^2 + Backstepping_error(2,:).^2 + Backstepping_error(3,:).^2).^0.5;
Backstepping_results.Backstepping_tracking_error = Backstepping_tracking_error;
Backstepping_results.Backstepping_tracking_error_norm = dt*sum(Backstepping_tracking_error,2);

% [tot_cost, track_cost, control_cost] = calculate_cost(Backstepping_results.error,Backstepping_results.U,dt);
% Backstepping_results.all_cost = [tot_cost, track_cost, control_cost];
end

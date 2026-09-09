function Backstepping_results = simulate_Backstepping(position, attitude, global_parameters, ref, IC, sim_params)
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
T = 0:dt:tf-dt;
N = length(T);
% Preallocating variables
X               = zeros(12,N);
U               = zeros(4, length(T));
X_ref           = zeros(12, length(T));
angles          = zeros(3,length(T));
r_yaw           = global_parameters.r_yaw;
U_pos            = zeros(3,N);     % here: commanded inertial accelerations (ax,ay,az - g style consistent w/ system_solve)
torques         = zeros(3,N);
att_error       = zeros(6,N);
pos_error       = zeros(6,N);
r_initial       = zeros(3,N-1);
instant_cost_pos = zeros(1, N);
cumulative_cost_pos = zeros(1, N);
instant_cost_att = zeros(1, N);
cumulative_cost_att = zeros(1, N);

%% wind sim stuff
Cd = [.8; 0.8;1.2];
L = 0.2;
h = 0.05;
A = [L*2*h, L*2*h, 2*L*L];
r = [ L,  L, -L, -L;
      L, -L, -L,  L;
      0,  0,  0,  0 ];
rho = 1.225;
Wxyz = gen_wind_vector(W_nominal,dt,tf);

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
Kv_pos = diag([3.5 3.5 4.5]);
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
    %% add payload uncertianty 10 times during simulation
    if payload_uncert_logic ==1  && mod(i,500) == 0
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
    %% reference at this timestep
    X_ref(1:6, i) = r_smooth(:,i);
    pd_dot  = X_ref(4:6, i);
    pd_ddot = r_xyz_ddot(:,i);
    %% 6x1 position error
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);

    %% OUTER LOOP BACKSTEPPING (position)
    e_p = pos_error(1:3,i);
    v_d = pd_dot - Kp_pos*e_p;
    e_v = X(4:6,i) - v_d;
    v_d_dot = pd_ddot - Kp_pos*(X(4:6,i) - pd_dot);
    a_cmd = v_d_dot - Kv_pos*e_v - Kc_pos*e_p;
    % a_max = (2*m*g)/m;     % = 2g from your max_Ft definition
    % a_cmd = max(min(a_cmd,  a_max), -a_max);  % elementwise clamp (simple)
    U_pos(:,i) = [a_cmd(1); a_cmd(2); a_cmd(3) - g];
    [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    
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


    % %% Assemble input
    % U(:,i) = [ft; torques(:,i)];
    % if saturation_logic == 1
    %     U(:,i) = saturate_controls(U(:,i));
    % end


    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end

    %% dynamics update
    f_x = Full_f(X(:,i), g, Ix, Iy, Iz);
    g_x = Full_g(X(:,i), m, Ix, Iy, Iz);

    if i < N
        X(:,i+1) = X(:,i) + dt*(f_x + g_x*U(:,i));
    end

    %% costs
    instant_cost_pos(i) = dt*( pos_error(:,i)'*Q_pos*pos_error(:,i) + U_pos(:,i)'*R_pos*U_pos(:,i) );
    instant_cost_att(i) = dt*( att_error(:,i)'*Q_att*att_error(:,i) + torques(:,i)'*R_att*torques(:,i) );

    if i == 1
        cumulative_cost_pos(i) = instant_cost_pos(i);
        cumulative_cost_att(i) = instant_cost_att(i);
    else
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
    end
end

Backstepping_sim_time = toc;

%% outputs 
%% save all useful info in a struck
Backstepping_results.X = X;
Backstepping_results.U = U;
Backstepping_results.X_ref = X_ref;
Backstepping_results.error = [pos_error;att_error];
Backstepping_results.inscost = [instant_cost_pos; instant_cost_att];
Backstepping_results.cumcost = [cumulative_cost_pos; cumulative_cost_att];
Backstepping_c_cost_total = sum(Backstepping_results.cumcost, 1);
Backstepping_results.Backstepping_c_cost_total = Backstepping_c_cost_total;
Backstepping_results.simtime = Backstepping_sim_time;
Backstepping_error = Backstepping_results.error;
Backstepping_tracking_error = (Backstepping_error(1,:).^2 + Backstepping_error(2,:).^2 + Backstepping_error(3,:).^2).^0.5;
Backstepping_results.Backstepping_tracking_error = Backstepping_tracking_error;
Backstepping_results.Backstepping_tracking_error_norm = dt*sum(Backstepping_tracking_error,2);

[tot_cost, track_cost, control_cost] = calculate_cost(Backstepping_results.error,Backstepping_results.U,dt);
Backstepping_results.all_cost = [tot_cost, track_cost, control_cost];
end

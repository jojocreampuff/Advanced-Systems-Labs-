function FL_results = simulate_FL(position, attitude, global_parameters, ref, IC, sim_params)
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

%% Feedback linearization gains (choose as 2nd-order linear error dynamics)
% Position: e_ddot + Kd_pos*e_dot + Kp_pos*e = 0
Kp_pos = diag([4 4 6]);     
Kd_pos = diag([4 4 5]);    

% Attitude outer->rate shaping (same role as Lambda_att in your SMC version)
Lambda_att = diag([8 8 4]); 

% Rate loop (feedback linearization with PD on omega tracking)
Kp_rate = diag([8 8 4]);
Kd_rate = diag([0 0 0]);

f_att_rate = @(ohm) [((Iy - Iz) / Ix * ohm(2) * ohm(3)); 
                    ((Iz - Ix) / Iy * ohm(1) * ohm(3)); 
                    ((Ix - Iy) / Iz * ohm(1) * ohm(2))];

g_att_rate = [1/Ix 0 0; 
              0 1/Iy 0; 
              0 0 1/Iz];

Trans = @(angles) [ 1,  sin(angles(1))*tan(angles(2)),  cos(angles(1))*tan(angles(2));
                    0,  cos(angles(1)),            -sin(angles(1));
                    0,  sin(angles(1))/cos(angles(2)),  cos(angles(1))/cos(angles(2))];

ohm_d = zeros(3,N);
omega_d_dot_filt = zeros(3,1);

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
% ESO GAINS
eso_state.p_hat = IC(1:3);
eso_state.v_hat = IC(4:6);
eso_state.d_hat = zeros(3,1);


ieso_state.p_hat = IC(1:3);
ieso_state.v_hat = IC(4:6);
ieso_state.d_hat = zeros(3,1);
use_ESO = 0;

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

    %% add wind disturbance
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), rho, Cd, A, r, m, Ix, Iy, Iz);
        X(:,i) = X(:,i)+ dt*[0;0;0;
                             axyz_uncert;
                             0;0;0;
                             alphaxyz_uncert];
    end

    %% Outer-loop: feedback-linearized position control (virtual acceleration command)
    % Reference pos/vel/acc
    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);

    % e, e_dot in inertial frame
    e_pos     = pos_error(1:3,i);
    e_pos_dot = pos_error(4:6,i);

    % Choose desired translational acceleration (in inertial frame):
    % x_ddot_cmd = x_ddot_ref - Kd*e_dot - Kp*e
    a_cmd = r_xyz_ddot(:,i) - Kd_pos*e_pos_dot - Kp_pos*e_pos;

    % Match your SMC convention: system_solve expects inputs consistent with "uz - g"
    % so we pass u3 = a_cmd(3) - g later. Here store as:
    U_pos(:,i) = [a_cmd(1); a_cmd(2); a_cmd(3)];
    %% ESO
    % if use_ESO == 1 && wind_logic == 1
    %     w0 = 150;  % try 5–15
    %     if i == 1
    %         U_prev = [m*g;0;0;0];  % hover guess
    %     else
    %         U_prev = U(:,i-1);     % applied control from previous step
    %     end
    % 
    %     [eso_state, d_hat_pos] = ESO_position_NEU(eso_state, X(:,i), U_prev, m, dt, w0);
    %     U_pos(:,i) = U_pos(:,i) - d_hat_pos;
    % end
    %% position Improved ESO from paper 
    if use_ESO == 1 && wind_logic == 1
        w0 = 100;  % try 5–15
        k_pos =.1*[1;1; 1];
        [ieso_state, d_hat_force, e_obs] = IESO_position_update(ieso_state, X(:,i), U_pos(:,i), m, dt, w0);

        % Improved compensation term: (d_hat + k*m*e_obs)/m
        U_pos(:,i) = U_pos(:,i) - (d_hat_force + (k_pos.*m).*e_obs)/m;
    end

    U_pos(:,i) = U_pos(:,i) - [0;0;g];

    %% Map acceleration command -> thrust + desired roll/pitch using your solver
    [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    %% Inner-loop: feedback linearization on rate dynamics (computed torque)
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);

    % Commanded body rate from Euler error shaping:
    % ohm_d = Trans(eta)^{-1} (eta_ref_dot - Lambda*e_eta)
    ohm_d(:,i) = Trans(X(7:9,i)) \ (X_ref(10:12, i) - Lambda_att * att_error(1:3,i));

    %% first order filter
    omega_d_dot_raw = angle_rate_solver(ohm_d,i,dt); % assumes this returns derivative of a vector input
    alpha = 0.9;
    omega_d_dot = alpha * omega_d_dot_filt + (1-alpha) * omega_d_dot_raw;
    omega_d_dot_filt = omega_d_dot;

    % Rate tracking error
    e_omega = X(10:12,i) - ohm_d(:,i);

    % Feedback linearization / computed torque:
    % omega_dot = f(omega) + g*tau
    % choose tau to make: e_omega_dot = -Kp_rate*e_omega   (or add -Kd_rate*e_omega_dot if you estimate it)
    v_rate = omega_d_dot - Kp_rate*e_omega;  % desired omega_dot

    % tau = g^{-1} ( v_rate - f(omega) )
    torques(:,i) = g_att_rate^-1 * ( v_rate - f_att_rate(X(10:12,i)) );

    %% Assemble input
    U(:,i) = [ft; torques(:,i)];

    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
    end

    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end

    %% Integrate dynamics
    f_x = Full_f(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g(X(:, i), m,Ix,Iy,Iz);

    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end

    instant_cost_pos(i) = dt * (pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos(:,i)' * R_pos * U_pos(:,i));
    instant_cost_att(i) = dt * (att_error(:,i)' * Q_att * att_error(:,i) + torques(:,i)' * R_att * torques(:,i));
    if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
    end
end

FL_sim_time = toc;

%% Pack results (same fields as your SMC function)
FL_results.X = X;
FL_results.U = U;
FL_results.X_ref = X_ref;
FL_results.error = [pos_error;att_error];
FL_results.inscost = [instant_cost_pos; instant_cost_att];
FL_results.cumcost = [cumulative_cost_pos; cumulative_cost_att];
FL_c_cost_total = sum(FL_results.cumcost, 1);
FL_results.FL_c_cost_total = FL_c_cost_total; % keep field name for easy comparison
FL_results.simtime = FL_sim_time;

FL_error = FL_results.error;
FL_tracking_error = (FL_error(1,:).^2 + FL_error(2,:).^2 + FL_error(3,:).^2).^0.5;
FL_results.FL_tracking_error = FL_tracking_error;            % keep field name
FL_results.FL_tracking_error_norm = dt*sum(FL_tracking_error,2);

[tot_cost, track_cost, control_cost] = calculate_cost(FL_results.error,FL_results.U,dt);
FL_results.all_cost = [tot_cost, track_cost, control_cost];

end

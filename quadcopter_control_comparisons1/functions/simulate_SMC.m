function SMC_results = simulate_SMC(position, attitude, global_parameters, ref, IC, sim_params)
%% added noise rejection to this SMC %%
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

%% sliding mode stuff
Cx = 1;     Cy = 1;     Cz = 1;
C_pos = .5*diag([Cx, Cy, Cz]);
Kx = 1;     Ky = 1;     Kz = 1;
K_xyz = 1*diag([Kx, Ky, Kz]);
eps_pos   = .5*[0.05; 0.05; 0.05];

Lambda_att = 2*diag([1 1 1]);
K_att = .5*diag([3 3 1.5]);
eps_att    = [0.05; 0.05; 0.05];

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
U               = zeros(4, length(T)); % Control input trajectory
X_ref           = zeros(12, length(T)); % reference trajectory
angles          = zeros(3,length(T));   % angles
r_yaw           = global_parameters.r_yaw; 
U_pos            = zeros(3,N);   % output of position SNAC
torques         = zeros(3,N);   % output of attitude SNAC
att_error       = zeros(6,N);   % error 
pos_error       = zeros(6,N);
r_initial       = zeros(3,N-1); % original trajectory
instant_cost_pos = zeros(1, N); % Instantaneous cost
cumulative_cost_pos = zeros(1, N); % Integrated cost
instant_cost_att = zeros(1, N); % Instantaneous cost
cumulative_cost_att = zeros(1, N); % Integrated cost

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

r_xyz_ddot = discrete_deriv(r_smooth(4:6,:),dt);

tic
%% ESO stuff
% ESO GAINS

use_ESO = 1;
if use_ESO == 1
    eso_att_state.omega_hat = X(10:12,1);   % initial body rate estimate
    eso_att_state.d_hat     = zeros(3,1);   % disturbance angular-accel estimate
    ieso_state.p_hat = IC(1:3);
    ieso_state.v_hat = IC(4:6);
    ieso_state.d_hat = zeros(3,1);
end

for i = 1:length(T)
    %% add payload uncertianty 10 times during simulation
    if payload_uncert_logic == 1 && mod(i,500) == 0
        m = inertia_params(1) + payload_uncert(1)*randn;
        Ix = inertia_params(2) + payload_uncert(2)*randn;
        Iy = inertia_params(3) + payload_uncert(3)*randn;
        Iz = inertia_params(4) + payload_uncert(4)*randn;
        g_att = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];
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

    %% SMC Position
    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i); % error is (x,y,z,u,v,w): [e_i;e_dot_i]
    S_pos = C_pos*pos_error(1:3,i) + pos_error(4:6,i); % C_pos*e + e_dot
    S_pos = max(-1, min(1, S_pos ./ eps_pos)); % bounrdy 
    U_pos(:,i) = r_xyz_ddot(:,i) - C_pos*pos_error(4:6,i) - K_xyz*S_pos; % desired accelerations in x y and z

    %% position Improved ESO from paper 
    if use_ESO == 1 && wind_logic == 1
        w0 = 15;  % try 5–15
        k_pos =.5*[1;1; 1];
        [ieso_state, d_hat_force, e_obs] = IESO_position_update(ieso_state, X(:,i), U_pos(:,i), m, dt, w0);
    
        % Improved compensation term: (d_hat + k*m*e_obs)/m
        U_pos(:,i) = U_pos(:,i) - (d_hat_force + (k_pos.*m).*e_obs)/m;
    end
    U_pos(:,i) = U_pos(:,i) - [0;0;g];
    %% system solver
    [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    %% SMC attitude controller
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    % commanded body rate
    ohm_d(:,i) = Trans(X(7:9,i)) \ (X_ref(10:12, i) - Lambda_att * att_error(1:3,i)); % ohm_d = T^-1 *(angle_ref_dot - gain*angle_error)

    %% first order filter
    omega_d_dot_raw = angle_rate_solver(ohm_d,i,dt);
    alpha = 0.9;  % filter factor (0.8-0.98)
    omega_d_dot = alpha * omega_d_dot_filt + (1-alpha) * omega_d_dot_raw;
    omega_d_dot_filt = omega_d_dot;
    
    % sliding variable
    s_att = X(10:12,i) - ohm_d(:,i);
    
    % sat(s/eps)
    sat_s = max(-1, min(1, s_att ./ eps_att));
   
    if use_ESO == 1 && wind_logic == 1
        w0_att = 20;
        k_att_obs = 1*[1; 1; 1];
        if i == 1
            tau_prev = [0;0;0];
        else
            tau_prev = U(2:4,i-1);   % previously applied torques (after saturation if you saturate U)
        end
    
        [eso_att_state, d_hat_att, e_obs_att] = IESO_attitude_update( ...
            eso_att_state, X(:,i), tau_prev, f_att_rate, g_att_rate, dt, w0_att);
    
        % subtract estimated disturbance (and optional observation-error compensation)
        % NOTE: d_hat_att is in rad/s^2, so in torque units it becomes g^{-1}*d_hat_att
        tau_eso_comp = g_att_rate^-1 * ( d_hat_att + k_att_obs .* e_obs_att );
    
    else
        tau_eso_comp = [0;0;0];
    end
    
    torques(:,i) = g_att_rate^-1 * (omega_d_dot - f_att_rate(X(10:12,i)) - K_att * sat_s) - tau_eso_comp;


    U(:,i) = [ft; torques(:,i)];

    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
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

    %% remember this is comtinous cost
    instant_cost_pos(i) = dt * (pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos(:,i)' * R_pos * U_pos(:,i));
    instant_cost_att(i) = dt * (att_error(:,i)' * Q_att * att_error(:,i) + torques(:,i)' * R_att * torques(:,i));
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
   end
end

SMC_sim_time = toc;
%% save all useful info in a struck
SMC_results.X = X;
SMC_results.U = U;
SMC_results.X_ref = X_ref;
SMC_results.error = [pos_error;att_error];
SMC_results.inscost = [instant_cost_pos; instant_cost_att];
SMC_results.cumcost = [cumulative_cost_pos; cumulative_cost_att];
SMC_c_cost_total = sum(SMC_results.cumcost, 1);
SMC_results.SMC_c_cost_total = SMC_c_cost_total;
SMC_results.simtime = SMC_sim_time;
SMC_error = SMC_results.error;
SMC_tracking_error = (SMC_error(1,:).^2 + SMC_error(2,:).^2 + SMC_error(3,:).^2).^0.5;
SMC_results.SMC_tracking_error = SMC_tracking_error;
SMC_results.SMC_tracking_error_norm = dt*sum(SMC_tracking_error,2);

[tot_cost, track_cost, control_cost] = calculate_cost(SMC_results.error,SMC_results.U,dt);
SMC_results.all_cost = [tot_cost, track_cost, control_cost];



% % if want to use attitude IESO 
% %% attitude IESO (improved ESO)

% if use_ESO == 1
%     if i == 1
%         tau_prev = [0;0;0];
%     else
%         tau_prev = U(2:4,i-1);   % previously applied torques (after saturation if you saturate U)
%     end
% 
%     [eso_att_state, d_hat_att, e_obs_att] = IESO_attitude_update( ...
%         eso_att_state, X(:,i), tau_prev, f_att_rate, g_att_rate, dt, w0_att);
% 
%     % improved compensation gain (like "k_phi, k_theta, k_psi" idea)
%     % keep it as vector so you can tune per-axis
%     k_att_obs = [k_phi; k_theta; k_psi];
% 
%     % subtract estimated disturbance (and optional observation-error compensation)
%     % NOTE: d_hat_att is in rad/s^2, so in torque units it becomes g^{-1}*d_hat_att
%     tau_eso_comp = g_att_rate^-1 * ( d_hat_att + k_att_obs .* e_obs_att );
% 
% else
%     tau_eso_comp = [0;0;0];
% end
% 
% torques(:,i) = g_att_rate^-1 * (omega_d_dot - f_att_rate(X(10:12,i)) - K_att * sat_s) - tau_eso_comp;
% 
% % If dt = 0.01: w0_att = 8 to 20
% start with k_phi = k_theta = k_psi = 0 (turn it off)
% 
% once it's stable, try small values like 0.5 to 3
function lqr_results = simulate_LQR(position, attitude, global_parameters, ref, IC, sim_params)
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

%% LQR stuff
[K_pos, K_att] = get_LQR_gains(1*Q_pos,1*R_pos, Q_att, R_att, Ix, Iy, Iz);

T = 0:dt:tf-dt;
N = length(T);

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
%% ESO stuff
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
    %% add wind disturbence
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), rho, Cd, A, r, m, Ix, Iy, Iz);
        X(:,i) = X(:,i)+ dt*[0;0;0;
                           axyz_uncert;
                           0;0;0;
                           alphaxyz_uncert];
    end

    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);

    U_pos(:,i) = -K_pos * (pos_error(:,i));

    %% ESO
    % if use_ESO == 1 && wind_logic == 1
    %     w0 = 50;  % try 5–15
    %     k_pos =.5*[1;1; 1];
    %     [ieso_state, d_hat_force, e_obs] = IESO_position_update(ieso_state, X(:,i), U_pos(:,i), m, dt, w0);
    % 
    %     % Improved compensation term: (d_hat + k*m*e_obs)/m
    %     U_pos(:,i) = U_pos(:,i) - (d_hat_force + (k_pos.*m).*e_obs)/m;
    % end
    % if use_ESO == 1 && wind_logic == 1
    %     w0 = 80;
    %     if i == 1
    %         U_prev = [m*g;0;0;0];  % hover guess
    %     else
    %         U_prev = U(:,i-1);     % applied control from previous step
    %     end
    % 
    %     [eso_state, d_hat_pos] = ESO_position(eso_state, X(:,i), U_prev, m, dt, w0);
    %     U_pos(:,i) = U_pos(:,i) - d_hat_pos;
    % end

    U_pos(:,i) = U_pos(:,i) + [0;0;g];
    [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i), -U_pos(3,i), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    % SNAC controller used to track angles - error regulation and optimal control equation
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = -K_att * (att_error(:,i));

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

    %% remember this is comtinous cost to discrete cost ADD dt term
%the main mistake I found 
    instant_cost_pos(i) = dt * (pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos(:,i)' * R_pos * U_pos(:,i));
    instant_cost_att(i) = dt * (att_error(:,i)' * Q_att * att_error(:,i) + torques(:,i)' * R_att * torques(:,i));
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
   end
end

lqr_sim_time = toc;
%% save all useful info in a struck
lqr_results.X = X;
lqr_results.U = U;
lqr_results.X_ref = X_ref;
lqr_results.error = [pos_error;att_error];
lqr_results.inscost = [instant_cost_pos; instant_cost_att];
lqr_results.cumcost = [cumulative_cost_pos; cumulative_cost_att];
lqr_c_cost_total = sum(lqr_results.cumcost, 1);
lqr_results.lqr_c_cost_total = lqr_c_cost_total;
lqr_results.simtime = lqr_sim_time;
lqr_results.gains = [K_att; K_pos];
lqr_error = lqr_results.error;
lqr_tracking_error = (lqr_error(1,:).^2 + lqr_error(2,:).^2 + lqr_error(3,:).^2).^0.5;
lqr_results.lqr_tracking_error = lqr_tracking_error;
lqr_results.lqr_tracking_error_norm = dt*sum(lqr_tracking_error,2);

[tot_cost, track_cost, control_cost] = calculate_cost(lqr_results.error,lqr_results.U,dt);
lqr_results.all_cost = [tot_cost, track_cost, control_cost];

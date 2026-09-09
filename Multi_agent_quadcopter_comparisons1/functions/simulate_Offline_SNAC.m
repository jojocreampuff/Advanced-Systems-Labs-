function Offline_SNAC_results = simulate_Offline_SNAC(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
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

%% offline SNAC shit
load('z_offline_SNAC_pos_data_MORE_R.mat','Position_W','Position_R')
load('z_offline_SNAC_att_NEW.mat', "Attitude_W", "Attitude_R","max_states")

Position_g =[0 0 0; 
    0 0 0; 
    0 0 0; 
    -1 0 0; 
    0 -1 0; 
    0 0 -1];
Position_G = Position_g*dt;

x4_max = max_states(4);
x5_max = max_states(5);
x6_max = max_states(6);
% this is for snacs non-dim stuff
ft_max = 2*m*g; % N
motor_max = ft_max/4; % 5 N per motor
drone_radius = .2; % lever Arm
u1_max = motor_max*2*drone_radius; 
u2_max = u1_max;
u3_max = u1_max;
Umax = [u1_max; u2_max; u3_max];
Attitude_g_bar = [0 0 0; 0 0 0; 0 0 0; u1_max/(Ix*x4_max) 0 0; 0 u2_max/(Iy*x5_max) 0; 0 0 u3_max/(Iz*x6_max)];
Attitude_G_bar = Attitude_g_bar * dt;

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
r_initial       = zeros(3,N); % original trajectory


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

    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);

    if mod(i-1,10) == 0
        %% romove the future look ahead
        % idx = min(i + 10, size(r_smooth,2));
        U_pos(:,i) =  -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(X(1:6, i)- r_smooth(:, i));
    
        [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i), -U_pos(3,i), r_yaw(i), m);
        angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
        X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];
    else
        U_pos(:,i) = U_pos(:,i-1);
        X_ref(7:12,i) = X_ref(7:12,i-1);
        angles(:,i) = angles(:,i-1);
    end
    % [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i), -U_pos(3,i), r_yaw(i), m);
    % angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    % X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    % SNAC controller used to track angles - error regulation and optimal control equation
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);

    torques(:,i) = -Attitude_R^-1 * Attitude_G_bar' * Attitude_W' * a_Offline_SNAC_phi_att(att_error(:,i)./max_states);
    torques(:,i) = torques(:,i).*Umax;

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
end

Offline_SNAC_sim_time = toc;
%% save all useful info in a struck
Offline_SNAC_results.X = X;
Offline_SNAC_results.U = U;
Offline_SNAC_results.X_ref = X_ref;
Offline_SNAC_results.U_pos = U_pos;
Offline_SNAC_results.error = [pos_error;att_error];
Offline_SNAC_results.simtime = Offline_SNAC_sim_time;
Offline_SNAC_error = Offline_SNAC_results.error;
Offline_SNAC_tracking_error = (Offline_SNAC_error(1,:).^2 + Offline_SNAC_error(2,:).^2 + Offline_SNAC_error(3,:).^2).^0.5;
Offline_SNAC_results.Offline_SNAC_tracking_error = Offline_SNAC_tracking_error;
Offline_SNAC_results.Offline_SNAC_tracking_error_norm = dt*sum(Offline_SNAC_tracking_error,2);

% [tot_cost, track_cost, control_cost] = calculate_cost(Offline_SNAC_results.error,Offline_SNAC_results.U,dt);
% Offline_SNAC_results.all_cost = [tot_cost, track_cost, control_cost];

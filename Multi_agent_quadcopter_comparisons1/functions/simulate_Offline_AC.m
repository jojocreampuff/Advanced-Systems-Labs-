function Offline_AC_results = simulate_Offline_AC(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
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

%% offline AC stuff
load("z_offline_AC_pos_MORE_R.mat","Vk")
Vk_pos = Vk;
load("z_offline_AC_att_NEW.mat", "Vk")
Vk_att = Vk;

% Preallocating variables
X               = zeros(12,N);
U               = zeros(4, length(T)); % Control input trajectory
X_ref           = zeros(12, length(T)); % reference trajectory
angles          = zeros(3,length(T));   % angles
r_yaw           = global_parameters.r_yaw; 
U_pos            = zeros(3,N);   % output of position AC
torques         = zeros(3,N);   % output of attitude AC
att_error       = zeros(6,N);   % error 
pos_error       = zeros(6,N);
r_initial       = zeros(3,N); % original trajector


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
        %% remove the future look ahead
        % idx = min(i + 10, size(r_smooth,2));
        r_future = r_smooth(:, i);
        pos_error_future = X(1:6, i) - r_future;
        U_pos(:,i) =  Vk_pos' * a_Offline_AC_phi(pos_error_future) - [0;0;g];
    
        [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i), r_yaw(i), m);
        angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
        X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];
    else
        U_pos(:,i) = U_pos(:,i-1);
        X_ref(7:12,i) = X_ref(7:12,i-1);
        angles(:,i) = angles(:,i-1);
    end

    % AC controller used to track angles - error regulation and optimal control equation
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = Vk_att'*a_Offline_AC_phi(att_error(:, i));

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

Offline_AC_sim_time = toc;
%% save all useful info in a struck
Offline_AC_results.X = X;
Offline_AC_results.U = U;
Offline_AC_results.X_ref = X_ref;
Offline_AC_results.U_pos = U_pos;
Offline_AC_results.error = [pos_error;att_error];
Offline_AC_results.simtime = Offline_AC_sim_time;
Offline_AC_error = Offline_AC_results.error;
Offline_AC_tracking_error = (Offline_AC_error(1,:).^2 + Offline_AC_error(2,:).^2 + Offline_AC_error(3,:).^2).^0.5;
Offline_AC_results.Offline_AC_tracking_error = Offline_AC_tracking_error;
Offline_AC_results.Offline_AC_tracking_error_norm = dt*sum(Offline_AC_tracking_error,2);


% [tot_cost, track_cost, control_cost] = calculate_cost(Offline_AC_results.error,Offline_AC_results.U,dt);
% Offline_AC_results.all_cost = [tot_cost, track_cost, control_cost];
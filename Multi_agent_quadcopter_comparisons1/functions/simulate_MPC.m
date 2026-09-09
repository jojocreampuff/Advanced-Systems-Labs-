function MPC_results = simulate_MPC(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
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
[~, K_att_lqr] = get_LQR_gains(10*Q_pos/dt,10*R_pos/dt, [300,300,100,10,10,10].*Q_att/dt, 1*R_att/dt, Ix, Iy, Iz);
%% MPC stuff
% MPC parameters
T_pos_horizon = 1.5;  % seconds
Hp = round(T_pos_horizon/dt);
pos_control_guess = zeros(3,Hp);

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
X_ref(1:6,:) = r_smooth;
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

    %% hold rate positon loop
    % use positon_horizion of 1 sec
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);
    if mod(i-1,10) == 0
        % remove the look ahead
        if i+Hp <= length(X_ref(1,:))
            pos_ref_horizon = X_ref(1:6, i : i+Hp);
        else
            pos_ref_horizon = pos_ref_horizon;
        end

        [pos_control_guess] = a_MPC_Pos_QP(pos_ref_horizon, X(1:6, i), pos_control_guess, Hp, Q_pos, R_pos, dt);
        pos_control_guess = pos_control_guess + [0;0;g];
        U_pos(:,i) = pos_control_guess(:,1);
        [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i), -U_pos(3,i), r_yaw(i), m);
        angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
        X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];
    else
        U_pos(:,i) = U_pos(:,i-1);
        X_ref(7:12,i) = X_ref(7:12,i-1);
        angles(:,i) = angles(:,i-1);
    end
    %% end of hold rate positon loop

    %% LQR attitude controller
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = -K_att_lqr * (att_error(:,i));

    U(:,i) = [ft; torques(:,i)];

    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
    end

    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end
    %% Update Nonlinear Dynamics using Euler integration
    f_x = Full_f(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g(X(:, i),m,Ix,Iy,Iz);
    
    %% State update using Euler integration
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end
end

MPC_sim_time = toc;
%% save all useful info in a struck
MPC_results.X = X;
MPC_results.U = U;
MPC_results.X_ref = X_ref;
MPC_results.error = [pos_error;att_error];
MPC_results.simtime = MPC_sim_time;
MPC_error = MPC_results.error;
MPC_tracking_error = (MPC_error(1,:).^2 + MPC_error(2,:).^2 + MPC_error(3,:).^2).^0.5;
MPC_results.MPC_tracking_error = MPC_tracking_error;
MPC_results.MPC_tracking_error_norm = dt*sum(MPC_tracking_error,2);

% [tot_cost, track_cost, control_cost] = calculate_cost(MPC_results.error,MPC_results.U,dt);
% MPC_results.all_cost = [tot_cost, track_cost, control_cost];

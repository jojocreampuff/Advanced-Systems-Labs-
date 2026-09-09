function Offline_SNAC_results = simulate_Offline_SNAC_4drone(seed,comm, position, attitude, global_parameters, ref, IC_all, sim_params, Wxyz)
rng(seed);
dt      = global_parameters.dt;
g       = global_parameters.g;
tf      = global_parameters.tf;
m       = global_parameters.m;
Ix      = global_parameters.Ix;
Iy      = global_parameters.Iy;
Iz      = global_parameters.Iz;
W_nominal = global_parameters.W_nominal;
state_noise = sim_params.state_noise;
control_noise = sim_params.control_noise;
saturation_logic = sim_params.saturation_logic;
wind_logic = sim_params.wind_logic;
reference_smoothing_logic = sim_params.reference_smoothing_logic;

T = 0:dt:tf-dt;
N = length(T);
Nd = 4;

r_yaw = global_parameters.r_yaw;

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
%% formation offsets
offsets = zeros(3, Nd);
offsets(:,1) = [ 0;  0; 0];
offsets(:,2) = [-1;  0; 0];
offsets(:,3) = [-1; -1; 0];
offsets(:,4) = [-1;  1; 0];
k_spring = 0;
k_damp = 0;
%% communication interval lengths
N1 = floor(N/3);
N2 = floor(2*N/3);
%% Preallocating variables
X               = zeros(12, N, Nd);   
U               = zeros(4,  N, Nd);   
X_ref           = zeros(12, N, Nd);   
angles          = zeros(3, N, Nd);    
U_pos           = zeros(3, N, Nd);    
torques         = zeros(3, N, Nd);    
att_error       = zeros(6, N, Nd);
pos_error       = zeros(6, N, Nd);
ft = zeros(Nd,1);
r_pitch = zeros(Nd,1);
r_roll = zeros(Nd,1);

tracking_error_xyz      = zeros(N, Nd);
tracking_error_xyz_norm = zeros(Nd, 1);

formation_error         = zeros(3, N, Nd);  % meaningful for drones 2,3,4
formation_error_norm    = zeros(N, Nd);
formation_error_int     = zeros(Nd, 1);

%% initial conditions
for d = 1:Nd
    X(:,1,d) = IC_all(:,d);
end

%% leader reference generation
for i = 1:N
    leader_ref_initial(:,i) = ref(T(i));
end

if reference_smoothing_logic == 1
    IC_pos = X(1:3,1,1)
    IC_vel = X(4:6,1,1)
    smooth_r_position_leader = smooth(leader_ref_initial, IC_pos', IC_vel', T);
else
    smooth_r_position_leader = leader_ref_initial;
end
r_smooth_leader = [smooth_r_position_leader; discrete_deriv(smooth_r_position_leader, dt)];
X_ref(1:6,:,1) = r_smooth_leader;

tic
for i = 1:N
    for d = 1:Nd
        % add noise and wind to all drones
        if state_noise ~= 0
            X(:,i,d) = add_state_noise(X(:,i,d), state_noise);
        end
        if wind_logic == 1
            [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i,d), m, Ix, Iy, Iz);
            X(:,i,d) = X(:,i,d) + dt * [0;0;0;axyz_uncert;0;0;0;alphaxyz_uncert];
        end
    end
%% different communication pathways
    if comm == 0
        X_ref(1:6,i,1) = r_smooth_leader(:,i); % all drones see the leader
        X_ref(1:6,i,2) = [X(1:3,i,1) + offsets(:,2); X(4:6,i,1)];
        X_ref(1:6,i,3) = [X(1:3,i,1) + offsets(:,3); X(4:6,i,1)];
        X_ref(1:6,i,4) = [X(1:3,i,1) + offsets(:,4); X(4:6,i,1)];
    elseif comm == 1
        if i <= N1/2
            X_ref(1:6,i,1) = r_smooth_leader(:,i); % all drones see the leader
            X_ref(1:6,i,2) = [X(1:3,i,1) + offsets(:,2); X(4:6,i,1)];
            X_ref(1:6,i,3) = [X(1:3,i,1) + offsets(:,3); X(4:6,i,1)];
            X_ref(1:6,i,4) = [X(1:3,i,1) + offsets(:,4); X(4:6,i,1)];
        elseif i <= N1 && i > N1/2
            X_ref(1:6,i,1) = r_smooth_leader(:,i); % leader drone reference
            X_ref(1:6,i,2) = [X(1:3,i,3) + (offsets(:,2) - offsets(:,3));   X(4:6,i,3)]; % 2 only sees 3
            X_ref(1:6,i,3) = [X(1:3,i,1) + offsets(:,3);                    X(4:6,i,1)]; % 3 only sees leader
            X_ref(1:6,i,4) = [X(1:3,i,2) + (offsets(:,4) - offsets(:,2));   X(4:6,i,2)]; % 4 only sees 2
        elseif i <= N2 && i > N1
            X_ref(1:6,i,1) = r_smooth_leader(:,i); % leader drone reference
            X_ref(1:6,i,2) = [X(1:3,i,1) + offsets(:,2);                    X(4:6,i,1)]; % 2 only sees leader
            X_ref(1:6,i,3) = [X(1:3,i,2) + (offsets(:,3) - offsets(:,2));   X(4:6,i,2)]; % 3 only sees 2
            X_ref(1:6,i,4) = [X(1:3,i,3) + (offsets(:,4) - offsets(:,3));   X(4:6,i,3)]; % 4 only sees 3
        else
            X_ref(1:6,i,1) = r_smooth_leader(:,i); % leader drone reference
            X_ref(1:6,i,2) = [X(1:3,i,3) + (offsets(:,2) - offsets(:,3));   X(4:6,i,3)]; % 2 only sees 3
            X_ref(1:6,i,3) = [(X(1:3,i,4) + (offsets(:,3)- offsets(:,4)));  X(4:6,i,4)]; % 3 only sees 4
            X_ref(1:6,i,4) = [X(1:3,i,1) + offsets(:,4);                    X(4:6,i,1)]; % 4 only sees leader
        end
    end
    pos_error(:,i,1) = X(1:6,i,1) - X_ref(1:6,i,1);
    pos_error(:,i,2) = X(1:6,i,2) - X_ref(1:6,i,2);
    pos_error(:,i,3) = X(1:6,i,3) - X_ref(1:6,i,3);
    pos_error(:,i,4) = X(1:6,i,4) - X_ref(1:6,i,4);
    
    if mod(i-1, 10) == 0

        U_pos(:,i,1) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(pos_error(:,i,1));
        U_pos(:,i,2) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(pos_error(:,i,2));
        U_pos(:,i,3) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(pos_error(:,i,3));
        U_pos(:,i,4) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(pos_error(:,i,4));

        [ft(1), r_pitch(1), r_roll(1)] = system_solve(-U_pos(1,i,1), -U_pos(2,i,1), -U_pos(3,i,1), r_yaw(i), m);
        [ft(2), r_pitch(2), r_roll(2)] = system_solve(-U_pos(1,i,2), -U_pos(2,i,2), -U_pos(3,i,2), r_yaw(i), m);
        [ft(3), r_pitch(3), r_roll(3)] = system_solve(-U_pos(1,i,3), -U_pos(2,i,3), -U_pos(3,i,3), r_yaw(i), m);
        [ft(4), r_pitch(4), r_roll(4)] = system_solve(-U_pos(1,i,4), -U_pos(2,i,4), -U_pos(3,i,4), r_yaw(i), m);

        angles(:,i,1) = [r_pitch(1); r_roll(1); r_yaw(i)];
        angles(:,i,2) = [r_pitch(2); r_roll(2); r_yaw(i)];
        angles(:,i,3) = [r_pitch(3); r_roll(3); r_yaw(i)];
        angles(:,i,4) = [r_pitch(4); r_roll(4); r_yaw(i)];

        X_ref(7:12,i,1) = [angles(:,i,1); angle_rate_solver(angles(:,:,1), i, dt)];
        X_ref(7:12,i,2) = [angles(:,i,2); angle_rate_solver(angles(:,:,2), i, dt)];
        X_ref(7:12,i,3) = [angles(:,i,3); angle_rate_solver(angles(:,:,3), i, dt)];
        X_ref(7:12,i,4) = [angles(:,i,4); angle_rate_solver(angles(:,:,4), i, dt)];
    else
        U_pos(:,i,1) = U_pos(:,i-1,1);
        U_pos(:,i,2) = U_pos(:,i-1,2);
        U_pos(:,i,3) = U_pos(:,i-1,3);
        U_pos(:,i,4) = U_pos(:,i-1,4);
        %% solve once every 10 timesteps (rates are held but look messy)
        angles(:,i,1) = angles(:,i-1,1);
        angles(:,i,2) = angles(:,i-1,2);
        angles(:,i,3) = angles(:,i-1,3);
        angles(:,i,4) = angles(:,i-1,4);

        X_ref(7:12,i,1) = X_ref(7:12,i-1,1);
        X_ref(7:12,i,2) = X_ref(7:12,i-1,2);
        X_ref(7:12,i,3) = X_ref(7:12,i-1,3);
        X_ref(7:12,i,4) = X_ref(7:12,i-1,4);
    end

    %% solve at every timestep (rates are only non-zeros every 10th timestep))
    % [ft(1), r_pitch(1), r_roll(1)] = system_solve(-U_pos(1,i,1), -U_pos(2,i,1), -U_pos(3,i,1), r_yaw(i), m);
    % [ft(2), r_pitch(2), r_roll(2)] = system_solve(-U_pos(1,i,2), -U_pos(2,i,2), -U_pos(3,i,2), r_yaw(i), m);
    % [ft(3), r_pitch(3), r_roll(3)] = system_solve(-U_pos(1,i,3), -U_pos(2,i,3), -U_pos(3,i,3), r_yaw(i), m);
    % [ft(4), r_pitch(4), r_roll(4)] = system_solve(-U_pos(1,i,4), -U_pos(2,i,4), -U_pos(3,i,4), r_yaw(i), m);
    % angles(:,i,1) = [r_pitch(1); r_roll(1); r_yaw(i)];
    % angles(:,i,2) = [r_pitch(2); r_roll(2); r_yaw(i)];
    % angles(:,i,3) = [r_pitch(3); r_roll(3); r_yaw(i)];
    % angles(:,i,4) = [r_pitch(4); r_roll(4); r_yaw(i)];
    % X_ref(7:12,i,1) = [angles(:,i,1); angle_rate_solver(angles(:,:,1), i, dt)];
    % X_ref(7:12,i,2) = [angles(:,i,2); angle_rate_solver(angles(:,:,2), i, dt)];
    % X_ref(7:12,i,3) = [angles(:,i,3); angle_rate_solver(angles(:,:,3), i, dt)];
    % X_ref(7:12,i,4) = [angles(:,i,4); angle_rate_solver(angles(:,:,4), i, dt)];

    att_error(:,i,1) = X(7:12,i,1) - X_ref(7:12,i,1);
    att_error(:,i,2) = X(7:12,i,2) - X_ref(7:12,i,2);
    att_error(:,i,3) = X(7:12,i,3) - X_ref(7:12,i,3);
    att_error(:,i,4) = X(7:12,i,4) - X_ref(7:12,i,4);

    torques(:,i,1) = (-Attitude_R^-1 * Attitude_G_bar' * ...
                         Attitude_W' * a_Offline_SNAC_phi_att(att_error(:,i,1)./max_states)) .* Umax;
    torques(:,i,2) = (-Attitude_R^-1 * Attitude_G_bar' * ...
                         Attitude_W' * a_Offline_SNAC_phi_att(att_error(:,i,2)./max_states)) .* Umax;
    torques(:,i,3) = (-Attitude_R^-1 * Attitude_G_bar' * ...
                         Attitude_W' * a_Offline_SNAC_phi_att(att_error(:,i,3)./max_states)) .* Umax;
    torques(:,i,4) = (-Attitude_R^-1 * Attitude_G_bar' * ...
                         Attitude_W' * a_Offline_SNAC_phi_att(att_error(:,i,4)./max_states)) .* Umax;

    U(:,i,1) = [ft(1); torques(:,i,1)];
    U(:,i,2) = [ft(2); torques(:,i,2)];
    U(:,i,3) = [ft(3); torques(:,i,3)];
    U(:,i,4) = [ft(4); torques(:,i,4)];

    if saturation_logic == 1
        U(:,i,1) = saturate_controls(U(:,i,1));
        U(:,i,2) = saturate_controls(U(:,i,2));
        U(:,i,3) = saturate_controls(U(:,i,3));
        U(:,i,4) = saturate_controls(U(:,i,4));
    end
    if control_noise ~= 0
        U(:,i,1) = add_control_noise(U(:,i,1), control_noise);
        U(:,i,2) = add_control_noise(U(:,i,2), control_noise);
        U(:,i,3) = add_control_noise(U(:,i,3), control_noise);
        U(:,i,4) = add_control_noise(U(:,i,4), control_noise);
    end

    for d = 1:Nd
        if i < N
            f_x = Full_f(X(:,i,d), g, Ix, Iy, Iz);
            g_x = Full_g(X(:,i,d), m, Ix, Iy, Iz);
            X(:,i+1,d) = X(:,i,d) + dt * (f_x + g_x * U(:,i,d));
        end
    end 

    %% costs
    for d = 1:Nd
        tracking_error_xyz(i,d) = norm(pos_error(1:3,i,d));
    end
    for d = 2:Nd
        formation_error(:,i,d) = X(1:3,i,d) - (X(1:3,i,1) + offsets(:,d));
        formation_error_norm(i,d) = norm(formation_error(:,i,d));
    end  
end

Offline_SNAC_sim_time = toc;

%% sum costs
for d = 1:Nd
    tracking_error_xyz_norm(d) = dt * sum(tracking_error_xyz(:,d));
    if d >= 2
        formation_error_int(d) = dt * sum(formation_error_norm(:,d));
    end
end


%% save outputs
Offline_SNAC_results.X = X;
Offline_SNAC_results.U = U;
Offline_SNAC_results.X_ref = X_ref;
Offline_SNAC_results.X1 = X(1:12,:,1);
Offline_SNAC_results.X2 = X(1:12,:,2);
Offline_SNAC_results.X3 = X(1:12,:,3);
Offline_SNAC_results.X4 = X(1:12,:,4);
Offline_SNAC_results.U1 = U(:,:,1);
Offline_SNAC_results.U2 = U(:,:,2);
Offline_SNAC_results.U3 = U(:,:,3);
Offline_SNAC_results.U4 = U(:,:,4);
Offline_SNAC_results.X1_ref = X_ref(:,:,1);
Offline_SNAC_results.X2_ref = X_ref(:,:,2);
Offline_SNAC_results.X3_ref = X_ref(:,:,3);
Offline_SNAC_results.X4_ref = X_ref(:,:,4);

Offline_SNAC_results.error.pos = pos_error;
Offline_SNAC_results.error.att = att_error;
Offline_SNAC_results.error.all = cat(1, pos_error, att_error);

Offline_SNAC_results.U_pos = U_pos;

Offline_SNAC_results.simtime = Offline_SNAC_sim_time;
Offline_SNAC_results.tracking_error = tracking_error_xyz';
Offline_SNAC_results.tracking_error_norm = tracking_error_xyz_norm;

Offline_SNAC_results.formation_error = formation_error;
Offline_SNAC_results.formation_error_norm = formation_error_norm;
Offline_SNAC_results.formation_error_int = formation_error_int;

Offline_SNAC_results.r_smooth1 = squeeze(X_ref(1:6,:,1));
Offline_SNAC_results.r_smooth2 = squeeze(X_ref(1:6,:,2));
Offline_SNAC_results.r_smooth3 = squeeze(X_ref(1:6,:,3));
Offline_SNAC_results.r_smooth4 = squeeze(X_ref(1:6,:,4));

Offline_SNAC_results.time = T;
Offline_SNAC_results.offsets = offsets;
end
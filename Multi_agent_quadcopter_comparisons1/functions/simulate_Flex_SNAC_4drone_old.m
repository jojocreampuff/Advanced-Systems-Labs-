function Flex_SNAC_results = simulate_Flex_SNAC_4drone_old(seed, comm, position, attitude, global_parameters, ref, IC_all, sim_params)
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
control_noise = sim_params.control_noise;
saturation_logic = sim_params.saturation_logic;
wind_logic = sim_params.wind_logic;
reference_smoothing_logic = sim_params.reference_smoothing_logic;

T = 0:dt:tf-dt;
N = length(T);
Nd = 4;

N1 = floor(N/3);
N2 = floor(2*N/3);
r_yaw = global_parameters.r_yaw;
%% load flexible attitude SNAC stuff
load("z_flex_att_fixed.mat","Attitude_R","Attitude_W")
Attitude_Rflex = Attitude_R;
Attitude_Wflex = Attitude_W;
Attitude_G = [0 0 0;
              0 0 0;
              0 0 0;
              1/Ix 0 0;
              0 1/Iy 0;
              0 0 1/Iz];

K = 10;   % lookahead update interval

%% offline SNAC shit
load('z_offline_SNAC_pos_data_MORE_R.mat','Position_W','Position_R')
Position_g =[0 0 0; 
    0 0 0; 
    0 0 0; 
    -1 0 0; 
    0 -1 0; 
    0 0 -1];
Position_G = Position_g*dt;
%% formation offsets
offsets = zeros(3, Nd);
offsets(:,1) = [ 0;  0; 0];
offsets(:,2) = [-1;  0; 0];
offsets(:,3) = [-1; -1; 0];
offsets(:,4) = [-1;  1; 0];

%% Preallocating variables
X               = zeros(12, N, Nd);
U               = zeros(4,  N, Nd);
X_ref           = zeros(12, N, Nd);
angles          = zeros(3,  N, Nd);
U_pos           = zeros(3,  N, Nd);
torques         = zeros(3,  N, Nd);
att_error       = zeros(6,  N, Nd);
pos_error       = zeros(6,  N, Nd);
ft              = zeros(Nd, N);
r_pitch         = zeros(Nd, 1);
r_roll          = zeros(Nd, 1);

tracking_error_xyz      = zeros(N, Nd);
tracking_error_xyz_norm = zeros(Nd, 1);

formation_error         = zeros(3, N, Nd);
formation_error_norm    = zeros(N, Nd);
formation_error_int     = zeros(Nd, 1);

leader_ref_initial = zeros(3, N);

temp = 1;

%% wind sim stuff
Cd = [.8; 0.8; 1.2];
L = 0.2;
h = 0.05;
A = [L*2*h, L*2*h, 2*L*L];
r = [ L,  L, -L, -L;
      L, -L, -L,  L;
      0,  0,  0,  0 ];
rho = 1.225;
Wxyz = gen_wind_vector(W_nominal, dt, tf);

%% initial conditions
X(:,1,1) = IC_all(:,1);
X(:,1,2) = IC_all(:,2);
X(:,1,3) = IC_all(:,3);
X(:,1,4) = IC_all(:,4);

%% leader reference
for i = 1:N
    leader_ref_initial(:,i) = ref(T(i));
end

if reference_smoothing_logic == 1
    smooth_r_position_leader = smooth(leader_ref_initial, X(1:3,1,1), X(4:6,1,1), T);
else
    smooth_r_position_leader = leader_ref_initial;
end
r_smooth_leader = [smooth_r_position_leader; discrete_deriv(smooth_r_position_leader, dt)];

tic
for i = 1:N
    if state_noise ~= 0
        X(:,i,1) = add_state_noise(X(:,i,1), state_noise);
        X(:,i,2) = add_state_noise(X(:,i,2), state_noise);
        X(:,i,3) = add_state_noise(X(:,i,3), state_noise);
        X(:,i,4) = add_state_noise(X(:,i,4), state_noise);
    end

    if wind_logic == 1
        [axyz_uncert1, alphaxyz_uncert1] = wind_f_m(Wxyz(:,i), X(:,i,1), rho, Cd, A, r, m, Ix, Iy, Iz);
        [axyz_uncert2, alphaxyz_uncert2] = wind_f_m(Wxyz(:,i), X(:,i,2), rho, Cd, A, r, m, Ix, Iy, Iz);
        [axyz_uncert3, alphaxyz_uncert3] = wind_f_m(Wxyz(:,i), X(:,i,3), rho, Cd, A, r, m, Ix, Iy, Iz);
        [axyz_uncert4, alphaxyz_uncert4] = wind_f_m(Wxyz(:,i), X(:,i,4), rho, Cd, A, r, m, Ix, Iy, Iz);

        axyz_uncert1 = axyz_uncert1(:); alphaxyz_uncert1 = alphaxyz_uncert1(:);
        axyz_uncert2 = axyz_uncert2(:); alphaxyz_uncert2 = alphaxyz_uncert2(:);
        axyz_uncert3 = axyz_uncert3(:); alphaxyz_uncert3 = alphaxyz_uncert3(:);
        axyz_uncert4 = axyz_uncert4(:); alphaxyz_uncert4 = alphaxyz_uncert4(:);

        X(:,i,1) = X(:,i,1) + dt * [0;0;0; axyz_uncert1; 0;0;0; alphaxyz_uncert1];
        X(:,i,2) = X(:,i,2) + dt * [0;0;0; axyz_uncert2; 0;0;0; alphaxyz_uncert2];
        X(:,i,3) = X(:,i,3) + dt * [0;0;0; axyz_uncert3; 0;0;0; alphaxyz_uncert3];
        X(:,i,4) = X(:,i,4) + dt * [0;0;0; axyz_uncert4; 0;0;0; alphaxyz_uncert4];
    end
comm=0;
    %% communication pathways
    if comm == 0
        X_ref(1:6,i,1) = r_smooth_leader(:,i);
        X_ref(1:6,i,2) = [X(1:3,i,1) + offsets(:,2); X(4:6,i,1)];
        X_ref(1:6,i,3) = [X(1:3,i,1) + offsets(:,3); X(4:6,i,1)];
        X_ref(1:6,i,4) = [X(1:3,i,1) + offsets(:,4); X(4:6,i,1)];
    elseif comm == 1
        if i <= N1/2
            X_ref(1:6,i,1) = r_smooth_leader(:,i);
            X_ref(1:6,i,2) = [X(1:3,i,1) + offsets(:,2); X(4:6,i,1)];
            X_ref(1:6,i,3) = [X(1:3,i,1) + offsets(:,3); X(4:6,i,1)];
            X_ref(1:6,i,4) = [X(1:3,i,1) + offsets(:,4); X(4:6,i,1)];
        elseif i <= N1
            X_ref(1:6,i,1) = r_smooth_leader(:,i);
            X_ref(1:6,i,2) = [X(1:3,i,3) + (offsets(:,2) - offsets(:,3)); X(4:6,i,3)];
            X_ref(1:6,i,3) = [X(1:3,i,1) + offsets(:,3); X(4:6,i,1)];
            X_ref(1:6,i,4) = [X(1:3,i,2) + (offsets(:,4) - offsets(:,2)); X(4:6,i,2)];
        elseif i <= N2
            X_ref(1:6,i,1) = r_smooth_leader(:,i);
            X_ref(1:6,i,2) = [X(1:3,i,1) + offsets(:,2); X(4:6,i,1)];
            X_ref(1:6,i,3) = [X(1:3,i,2) + (offsets(:,3) - offsets(:,2)); X(4:6,i,2)];
            X_ref(1:6,i,4) = [X(1:3,i,3) + (offsets(:,4) - offsets(:,3)); X(4:6,i,3)];
        else
            X_ref(1:6,i,1) = r_smooth_leader(:,i);
            X_ref(1:6,i,2) = [X(1:3,i,3) + (offsets(:,2) - offsets(:,3)); X(4:6,i,3)];
            X_ref(1:6,i,3) = [X(1:3,i,4) + (offsets(:,3) - offsets(:,4)); X(4:6,i,4)];
            X_ref(1:6,i,4) = [X(1:3,i,1) + offsets(:,4); X(4:6,i,1)];
        end
    end

    pos_error(:,i,1) = X(1:6,i,1) - X_ref(1:6,i,1);
    pos_error(:,i,2) = X(1:6,i,2) - X_ref(1:6,i,2);
    pos_error(:,i,3) = X(1:6,i,3) - X_ref(1:6,i,3);
    pos_error(:,i,4) = X(1:6,i,4) - X_ref(1:6,i,4);

    if mod(i-1, 1) == 0
        r_future1 = r_smooth_leader(:,i);
        r_future2 = X_ref(1:6,i,2);
        r_future3 = X_ref(1:6,i,3);
        r_future4 = X_ref(1:6,i,4);

        U_pos(:,i,1) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(X(1:6,i,1) - r_future1);
        U_pos(:,i,2) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(X(1:6,i,2) - r_future2);
        U_pos(:,i,3) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(X(1:6,i,3) - r_future3);
        U_pos(:,i,4) = -Position_R^-1 * Position_G' * Position_W' * a_Offline_SNAC_phi_pos(X(1:6,i,4) - r_future4);

        [ft(1,i), r_pitch(1), r_roll(1)] = system_solve(-U_pos(1,i,1), -U_pos(2,i,1), -U_pos(3,i,1), r_yaw(i), m);
        [ft(2,i), r_pitch(2), r_roll(2)] = system_solve(-U_pos(1,i,2), -U_pos(2,i,2), -U_pos(3,i,2), r_yaw(i), m);
        [ft(3,i), r_pitch(3), r_roll(3)] = system_solve(-U_pos(1,i,3), -U_pos(2,i,3), -U_pos(3,i,3), r_yaw(i), m);
        [ft(4,i), r_pitch(4), r_roll(4)] = system_solve(-U_pos(1,i,4), -U_pos(2,i,4), -U_pos(3,i,4), r_yaw(i), m);

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

        ft(1,i) = ft(1,i-1);
        ft(2,i) = ft(2,i-1);
        ft(3,i) = ft(3,i-1);
        ft(4,i) = ft(4,i-1);

        angles(:,i,1) = angles(:,i-1,1);
        angles(:,i,2) = angles(:,i-1,2);
        angles(:,i,3) = angles(:,i-1,3);
        angles(:,i,4) = angles(:,i-1,4);

        X_ref(7:12,i,1) = X_ref(7:12,i-1,1);
        X_ref(7:12,i,2) = X_ref(7:12,i-1,2);
        X_ref(7:12,i,3) = X_ref(7:12,i-1,3);
        X_ref(7:12,i,4) = X_ref(7:12,i-1,4);
    end

    att_error(:,i,1) = X(7:12,i,1) - X_ref(7:12,i,1);
    att_error(:,i,2) = X(7:12,i,2) - X_ref(7:12,i,2);
    att_error(:,i,3) = X(7:12,i,3) - X_ref(7:12,i,3);
    att_error(:,i,4) = X(7:12,i,4) - X_ref(7:12,i,4);

    torques(:,i,1) = -Attitude_Rflex^-1 * Attitude_G' * Attitude_Wflex(:,:,temp)' * a_phi_att_flex(att_error(:,i,1));
    torques(:,i,2) = -Attitude_Rflex^-1 * Attitude_G' * Attitude_Wflex(:,:,temp)' * a_phi_att_flex(att_error(:,i,2));
    torques(:,i,3) = -Attitude_Rflex^-1 * Attitude_G' * Attitude_Wflex(:,:,temp)' * a_phi_att_flex(att_error(:,i,3));
    torques(:,i,4) = -Attitude_Rflex^-1 * Attitude_G' * Attitude_Wflex(:,:,temp)' * a_phi_att_flex(att_error(:,i,4));

    U(:,i,1) = [ft(1,i); torques(:,i,1)];
    U(:,i,2) = [ft(2,i); torques(:,i,2)];
    U(:,i,3) = [ft(3,i); torques(:,i,3)];
    U(:,i,4) = [ft(4,i); torques(:,i,4)];

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

    if i < N
        f_x1 = Full_f(X(:, i, 1), g, Ix, Iy, Iz);
        g_x1 = Full_g(X(:, i, 1), m, Ix, Iy, Iz);
        X(:, i+1, 1) = X(:, i, 1) + dt * (f_x1 + g_x1 * U(:, i, 1));

        f_x2 = Full_f(X(:, i, 2), g, Ix, Iy, Iz);
        g_x2 = Full_g(X(:, i, 2), m, Ix, Iy, Iz);
        X(:, i+1, 2) = X(:, i, 2) + dt * (f_x2 + g_x2 * U(:, i, 2));

        f_x3 = Full_f(X(:, i, 3), g, Ix, Iy, Iz);
        g_x3 = Full_g(X(:, i, 3), m, Ix, Iy, Iz);
        X(:, i+1, 3) = X(:, i, 3) + dt * (f_x3 + g_x3 * U(:, i, 3));

        f_x4 = Full_f(X(:, i, 4), g, Ix, Iy, Iz);
        g_x4 = Full_g(X(:, i, 4), m, Ix, Iy, Iz);
        X(:, i+1, 4) = X(:, i, 4) + dt * (f_x4 + g_x4 * U(:, i, 4));
    end

    tracking_error_xyz(i,1) = norm(pos_error(1:3,i,1));
    tracking_error_xyz(i,2) = norm(pos_error(1:3,i,2));
    tracking_error_xyz(i,3) = norm(pos_error(1:3,i,3));
    tracking_error_xyz(i,4) = norm(pos_error(1:3,i,4));

    formation_error(:,i,2) = X(1:3,i,2) - (X(1:3,i,1) + offsets(:,2));
    formation_error(:,i,3) = X(1:3,i,3) - (X(1:3,i,1) + offsets(:,3));
    formation_error(:,i,4) = X(1:3,i,4) - (X(1:3,i,1) + offsets(:,4));

    formation_error_norm(i,2) = norm(formation_error(:,i,2));
    formation_error_norm(i,3) = norm(formation_error(:,i,3));
    formation_error_norm(i,4) = norm(formation_error(:,i,4));

    temp = temp + 1;
    if temp > size(Attitude_Wflex,3)
        temp = 1;
    end
end

Flex_SNAC_sim_time = toc;

tracking_error_xyz_norm(1) = dt * sum(tracking_error_xyz(:,1));
tracking_error_xyz_norm(2) = dt * sum(tracking_error_xyz(:,2));
tracking_error_xyz_norm(3) = dt * sum(tracking_error_xyz(:,3));
tracking_error_xyz_norm(4) = dt * sum(tracking_error_xyz(:,4));

formation_error_int(2) = dt * sum(formation_error_norm(:,2));
formation_error_int(3) = dt * sum(formation_error_norm(:,3));
formation_error_int(4) = dt * sum(formation_error_norm(:,4));

Flex_SNAC_results.X = X;
Flex_SNAC_results.U = U;
Flex_SNAC_results.X_ref = X_ref;
Flex_SNAC_results.X1 = X(:,:,1);
Flex_SNAC_results.X2 = X(:,:,2);
Flex_SNAC_results.X3 = X(:,:,3);
Flex_SNAC_results.X4 = X(:,:,4);

Flex_SNAC_results.U1 = U(:,:,1);
Flex_SNAC_results.U2 = U(:,:,2);
Flex_SNAC_results.U3 = U(:,:,3);
Flex_SNAC_results.U4 = U(:,:,4);

Flex_SNAC_results.X1_ref = X_ref(:,:,1);
Flex_SNAC_results.X2_ref = X_ref(:,:,2);
Flex_SNAC_results.X3_ref = X_ref(:,:,3);
Flex_SNAC_results.X4_ref = X_ref(:,:,4);

Flex_SNAC_results.error.pos = pos_error;
Flex_SNAC_results.error.att = att_error;
Flex_SNAC_results.error.all = cat(1, pos_error, att_error);

Flex_SNAC_results.U1_pos = U_pos(:,:,1);
Flex_SNAC_results.torques = torques;
Flex_SNAC_results.angles = angles;

Flex_SNAC_results.simtime = Flex_SNAC_sim_time;
Flex_SNAC_results.tracking_error = tracking_error_xyz';
Flex_SNAC_results.tracking_error_norm = tracking_error_xyz_norm;

Flex_SNAC_results.formation_error = formation_error;
Flex_SNAC_results.formation_error_norm = formation_error_norm;
Flex_SNAC_results.formation_error_int = formation_error_int;

Flex_SNAC_results.r_smooth1 = squeeze(X_ref(1:6,:,1));
Flex_SNAC_results.r_smooth2 = squeeze(X_ref(1:6,:,2));
Flex_SNAC_results.r_smooth3 = squeeze(X_ref(1:6,:,3));
Flex_SNAC_results.r_smooth4 = squeeze(X_ref(1:6,:,4));

Flex_SNAC_results.time = T;
Flex_SNAC_results.offsets = offsets;
end
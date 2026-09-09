function SMC_results = simulate_SMC_4drone(seed,comm, position, attitude, global_parameters, ref, IC_all, sim_params, Wxyz)
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
r_yaw = global_parameters.r_yaw;

%% sliding mode stuff
% Cx = 1; Cy = 1; Cz = 1.5;
% C_pos = .3*diag([Cx, Cy, Cz]);
% 
% Kx = .5; Ky = .5; Kz = .8;
% K_xyz = 2.0*diag([Kx, Ky, Kz]);
% 
% eps_pos = 2*[0.05; 0.05; 0.05];
%% wind gains
Cx = 1;     Cy = 1;     Cz = 1.5;
C_pos = .5*diag([Cx, Cy, Cz]);
Kx = 1.0;     Ky = .5;     Kz = .8;
K_xyz = 4*diag([Kx, Ky, Kz]);
eps_pos   = 2*[0.05; 0.05; 0.05];

Lambda_att = 2*diag([1 1 1]);
K_att = .5*diag([3 3 1.5]);
eps_att = [0.05; 0.05; 0.05];

f_att_rate = @(ohm) [((Iy - Iz) / Ix * ohm(2) * ohm(3));
                     ((Iz - Ix) / Iy * ohm(1) * ohm(3));
                     ((Ix - Iy) / Iz * ohm(1) * ohm(2))];

g_att_rate = [1/Ix 0 0;
              0 1/Iy 0;
              0 0 1/Iz];

Trans = @(angles) [1,  sin(angles(1))*tan(angles(2)),  cos(angles(1))*tan(angles(2));
                   0,  cos(angles(1)),               -sin(angles(1));
                   0,  sin(angles(1))/cos(angles(2)), cos(angles(1))/cos(angles(2))];

%% formation offsets
offsets = zeros(3, Nd);
offsets(:,1) = [ 0;  0; 0];
offsets(:,2) = [-1;  0; 0];
offsets(:,3) = [-1; -1; 0];
offsets(:,4) = [-1;  1; 0];
%% communication interval lengths
N1 = floor(N/3);
N2 = floor(2*N/3);

%% prealloc
X         = zeros(12, N, Nd);
U         = zeros(4,  N, Nd);
X_ref     = zeros(12, N, Nd);
angles    = zeros(3,  N, Nd);
U_pos     = zeros(3,  N, Nd);
torques   = zeros(3,  N, Nd);
att_error = zeros(6,  N, Nd);
pos_error = zeros(6,  N, Nd);
ft        = zeros(Nd, 1);

tracking_error_xyz      = zeros(N, Nd);
tracking_error_xyz_norm = zeros(Nd, 1);
formation_error         = zeros(3, N, Nd);
formation_error_norm    = zeros(N, Nd);
formation_error_int     = zeros(Nd, 1);

leader_ref_initial = zeros(3, N);
ohm_d = zeros(3, N, Nd);
omega_d_dot_filt = zeros(3, Nd);

%% initial conditions
for d = 1:Nd
    X(:,1,d) = IC_all(:,d);
end

%% leader reference
for i = 1:N
    leader_ref_initial(:,i) = ref(T(i));
end

if reference_smoothing_logic == 1
    smooth_r_position_leader = smooth(leader_ref_initial, X(1:3,1,1), X(4:6,1,1), T);
else
    smooth_r_position_leader = leader_ref_initial;
end

r_smooth_leader = [smooth_r_position_leader; discrete_deriv(smooth_r_position_leader,dt)];
r_xyz_ddot_leader = discrete_deriv(r_smooth_leader(4:6,:),dt);

%% ESO
use_ESO = 1;
if use_ESO == 1
    for d = 1:Nd
        eso_att_state(d).omega_hat = X(10:12,1,d);
        eso_att_state(d).d_hat     = zeros(3,1);

        ieso_state(d).p_hat = X(1:3,1,d);
        ieso_state(d).v_hat = X(4:6,1,d);
        ieso_state(d).d_hat = zeros(3,1);
    end
end

tic
for i = 1:N
    %% noise and wind
    for d = 1:Nd
        if state_noise ~= 0
            X(:,i,d) = add_state_noise(X(:,i,d), state_noise);
        end
        if wind_logic == 1
            [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i,d),  m, Ix, Iy, Iz);
            X(:,i,d) = X(:,i,d) + dt*[0;0;0; axyz_uncert; 0;0;0; alphaxyz_uncert];
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
            X_ref(1:6,i,3) = [X(1:3,i,1) + offsets(:,3);                    X(4:6,i,1)]; % 2 only sees leader
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

    for d = 1:Nd
        pos_error(:,i,d) = X(1:6,i,d) - X_ref(1:6,i,d);
    end

    %% held position loop
    if mod(i-1,10) == 0
        for d = 1:Nd
            S_pos = C_pos*pos_error(1:3,i,d) + pos_error(4:6,i,d);
            S_pos = max(-1, min(1, S_pos ./ eps_pos));

            if d == 1
                U_pos(:,i,d) = r_xyz_ddot_leader(:,i) - C_pos*pos_error(4:6,i,d) - K_xyz*S_pos;
            else
                U_pos(:,i,d) = - C_pos*pos_error(4:6,i,d) - K_xyz*S_pos;
            end

            if use_ESO == 1 && wind_logic == 1
                w0 = 1;
                k_pos = .1*[1;1;1];
                [ieso_state(d), d_hat_force, e_obs] = IESO_position_update(ieso_state(d), X(:,i,d), U_pos(:,i,d), m, 10*dt, w0);
                U_pos(:,i,d) = U_pos(:,i,d) - (d_hat_force + (k_pos.*m).*e_obs)/m;
            end
            %% solve once every 10 timesteps (rates are held but look messy)
            [ft(d), r_pitch, r_roll] = system_solve(U_pos(1,i,d), U_pos(2,i,d), U_pos(3,i,d)-g, r_yaw(i), m);
            angles(:,i,d) = [r_pitch; r_roll; r_yaw(i)];
            X_ref(7:12,i,d) = [angles(:,i,d); angle_rate_solver(angles(:,:,d), i, dt)];
        end
    else
        for d = 1:Nd
            U_pos(:,i,d) = U_pos(:,i-1,d);

            angles(:,i,d) = angles(:,i-1,d);

            X_ref(7:12,i,d) = X_ref(7:12,i-1,d);
        end
    end

     %% solve at every timestep (rates are only non-zeros every 10th timestep))
    % %% system solve + attitude ref every step
    % for d = 1:Nd
    %     [ft(d,i), r_pitch, r_roll] = system_solve(U_pos(1,i,d), U_pos(2,i,d), U_pos(3,i,d)-g, r_yaw(i), m);
    %     angles(:,i,d) = [r_pitch; r_roll; r_yaw(i)];
    %     X_ref(7:12,i,d) = [angles(:,i,d); angle_rate_solver(angles(:,:,d), i, dt)];
    % end

    %% attitude loop
    for d = 1:Nd
        att_error(:,i,d) = X(7:12,i,d) - X_ref(7:12,i,d);

        ohm_d(:,i,d) = Trans(X(7:9,i,d)) \ (X_ref(10:12,i,d) - Lambda_att * att_error(1:3,i,d));

        omega_d_dot_raw = angle_rate_solver(ohm_d(:,:,d), i, dt);
        alpha = 0.9;
        omega_d_dot = alpha * omega_d_dot_filt(:,d) + (1-alpha) * omega_d_dot_raw;
        omega_d_dot_filt(:,d) = omega_d_dot;

        s_att = X(10:12,i,d) - ohm_d(:,i,d);
        sat_s = max(-1, min(1, s_att ./ eps_att));

        if use_ESO == 1 && wind_logic == 1
            w0_att = 20;
            k_att_obs = [1;1;1];

            if i == 1
                tau_prev = [0;0;0];
            else
                tau_prev = U(2:4,i-1,d);
            end

            [eso_att_state(d), d_hat_att, e_obs_att] = IESO_attitude_update( ...
                eso_att_state(d), X(:,i,d), tau_prev, f_att_rate, g_att_rate, dt, w0_att);

            tau_eso_comp = g_att_rate^-1 * (d_hat_att + k_att_obs .* e_obs_att);
        else
            tau_eso_comp = [0;0;0];
        end

        torques(:,i,d) = g_att_rate^-1 * (omega_d_dot - f_att_rate(X(10:12,i,d)) - K_att * sat_s) - tau_eso_comp;
        U(:,i,d) = [ft(d); torques(:,i,d)];

        if saturation_logic == 1
            U(:,i,d) = saturate_controls(U(:,i,d));
        end

        if control_noise ~= 0
            U(:,i,d) = add_control_noise(U(:,i,d), control_noise);
        end
    end

    %% update
    for d = 1:Nd
        if i < N
            f_x = Full_f(X(:,i,d), g, Ix, Iy, Iz);
            g_x = Full_g(X(:,i,d), m, Ix, Iy, Iz);
            X(:,i+1,d) = X(:,i,d) + dt * (f_x + g_x * U(:,i,d));
        end
    end

    %% metrics
    for d = 1:Nd
        tracking_error_xyz(i,d) = norm(pos_error(1:3,i,d));
    end
    for d = 2:Nd
        formation_error(:,i,d) = X(1:3,i,d) - (X(1:3,i,1) + offsets(:,d));
        formation_error_norm(i,d) = norm(formation_error(:,i,d));
    end
end

SMC_sim_time = toc;

for d = 1:Nd
    tracking_error_xyz_norm(d) = dt * sum(tracking_error_xyz(:,d));
    if d >= 2
        formation_error_int(d) = dt * sum(formation_error_norm(:,d));
    end
end

SMC_results.X = X;
SMC_results.U = U;
SMC_results.X_ref = X_ref;
SMC_results.error.pos = pos_error;
SMC_results.error.att = att_error;
SMC_results.error.all = cat(1, pos_error, att_error);
SMC_results.U_pos = U_pos;
SMC_results.simtime = SMC_sim_time;
SMC_results.tracking_error = tracking_error_xyz';
SMC_results.tracking_error_norm = tracking_error_xyz_norm;
SMC_results.formation_error = formation_error;
SMC_results.formation_error_norm = formation_error_norm;
SMC_results.formation_error_int = formation_error_int;
SMC_results.r_smooth1 = squeeze(X_ref(1:6,:,1));
SMC_results.r_smooth2 = squeeze(X_ref(1:6,:,2));
SMC_results.r_smooth3 = squeeze(X_ref(1:6,:,3));
SMC_results.r_smooth4 = squeeze(X_ref(1:6,:,4));
SMC_results.time = T;
SMC_results.offsets = offsets;
end
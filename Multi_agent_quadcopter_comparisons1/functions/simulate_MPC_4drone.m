function MPC_results = simulate_MPC_4drone(seed,comm,position, attitude, global_parameters, ref, IC_all, sim_params, Wxyz)
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
%% communication interval lengths
N1 = floor(N/3);
N2 = floor(2*N/3);
r_yaw = global_parameters.r_yaw;

[~, K_att_MPC] = get_LQR_gains(10*Q_pos/dt,10*R_pos/dt, [300,300,100,10,10,10].*Q_att/dt, 1*R_att/dt, Ix, Iy, Iz);

%% MPC stuff
T_pos_horizon = .8;
T_pos_folower = .5;
Hp = round(T_pos_horizon/dt);
pos_control_guess = zeros(3, Hp, Nd);

%% Formation offsets
offsets = zeros(3, Nd);
offsets(:,1) = [ 0;  0; 0];
offsets(:,2) = [-1;  0; 0];
offsets(:,3) = [-1; -1; 0];
offsets(:,4) = [-1;  1; 0];

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
    smooth_r_position_leader = smooth(leader_ref_initial, X(1:3,1,1), X(4:6,1,1), T);
else
    smooth_r_position_leader = leader_ref_initial;
end
r_smooth_leader = [smooth_r_position_leader; discrete_deriv(smooth_r_position_leader, dt)];
X_ref(1:6,:,1) = r_smooth_leader;

tic
for i = 1:length(T)
    for d = 1:Nd
        % add noise and wind to all drones
        if state_noise ~= 0
            X(:,i,d) = add_state_noise(X(:,i,d), state_noise);
        end
        if wind_logic == 1
            [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i,d),m, Ix, Iy, Iz);
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


    pos_error(:,i,1) = X(1:6,i,1) - X_ref(1:6,i,1);
    pos_error(:,i,2) = X(1:6,i,2) - X_ref(1:6,i,2);
    pos_error(:,i,3) = X(1:6,i,3) - X_ref(1:6,i,3);
    pos_error(:,i,4) = X(1:6,i,4) - X_ref(1:6,i,4);
    if mod(i-1, 10) == 0
        if i+Hp<= length(X_ref(1,:,1))
            pos_ref_horizon1 = X_ref(1:6, i : i+Hp, 1);
            pos_ref_horizon2 = X_ref(1:6,i,2).*ones(6,length(pos_ref_horizon1(1,:)));
            pos_ref_horizon3 = X_ref(1:6,i,3).*ones(6,length(pos_ref_horizon1(1,:)));
            pos_ref_horizon4 = X_ref(1:6,i,4).*ones(6,length(pos_ref_horizon1(1,:)));
        else
            pos_ref_horizon1 = pos_ref_horizon1;
            pos_ref_horizon2 = pos_ref_horizon2;
            pos_ref_horizon3 = pos_ref_horizon3;
            pos_ref_horizon4 = pos_ref_horizon4;
        end
    
        pos_control_guess(:,:,1) = a_MPC_Pos_QP(pos_ref_horizon1,X(1:6, i, 1), pos_control_guess(:,:,1),Hp, Q_pos, R_pos, dt);
        pos_control_guess(:,:,2) = a_MPC_Pos_QP(pos_ref_horizon2,X(1:6, i, 2), pos_control_guess(:,:,2),Hp, Q_pos, R_pos, dt);
        pos_control_guess(:,:,3) = a_MPC_Pos_QP(pos_ref_horizon3,X(1:6, i, 3), pos_control_guess(:,:,3),Hp, Q_pos, R_pos, dt);
        pos_control_guess(:,:,4) = a_MPC_Pos_QP(pos_ref_horizon4,X(1:6, i, 4), pos_control_guess(:,:,4),Hp, Q_pos, R_pos, dt);
    
        pos_control_guess(:,:,1) = pos_control_guess(:,:,1) + [0;0;g];
        U_pos(:,i,1) = pos_control_guess(:,1,1);
        pos_control_guess(:,:,2) = pos_control_guess(:,:,2) + [0;0;g];
        U_pos(:,i,2) = pos_control_guess(:,1,2);
        pos_control_guess(:,:,3) = pos_control_guess(:,:,3) + [0;0;g];
        U_pos(:,i,3) = pos_control_guess(:,1,3);
        pos_control_guess(:,:,4) = pos_control_guess(:,:,4) + [0;0;g];
        U_pos(:,i,4) = pos_control_guess(:,1,4);
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
    % 
    % angles(:,i,1) = [r_pitch(1); r_roll(1); r_yaw(i)];
    % angles(:,i,2) = [r_pitch(2); r_roll(2); r_yaw(i)];
    % angles(:,i,3) = [r_pitch(3); r_roll(3); r_yaw(i)];
    % angles(:,i,4) = [r_pitch(4); r_roll(4); r_yaw(i)];
    % 
    % X_ref(7:12,i,1) = [angles(:,i,1); angle_rate_solver(angles(:,:,1), i, dt)];
    % X_ref(7:12,i,2) = [angles(:,i,2); angle_rate_solver(angles(:,:,2), i, dt)];
    % X_ref(7:12,i,3) = [angles(:,i,3); angle_rate_solver(angles(:,:,3), i, dt)];
    % X_ref(7:12,i,4) = [angles(:,i,4); angle_rate_solver(angles(:,:,4), i, dt)];

    att_error(:,i,1) = X(7:12,i,1) - X_ref(7:12,i,1);
    att_error(:,i,2) = X(7:12,i,2) - X_ref(7:12,i,2);
    att_error(:,i,3) = X(7:12,i,3) - X_ref(7:12,i,3);
    att_error(:,i,4) = X(7:12,i,4) - X_ref(7:12,i,4);

    torques(:,i,1) = -K_att_MPC * att_error(:,i,1);
    torques(:,i,2) = -K_att_MPC * att_error(:,i,2);
    torques(:,i,3) = -K_att_MPC * att_error(:,i,3);
    torques(:,i,4) = -K_att_MPC * att_error(:,i,4);

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
MPC_sim_time = toc;

%% sum costs
for d = 1:Nd
    tracking_error_xyz_norm(d) = dt * sum(tracking_error_xyz(d,:));
    if d >= 2
        formation_error_int(d) = dt * sum(formation_error_norm(d,:));
    end
end

%% save outputs
MPC_results.X = X;
MPC_results.U = U;
MPC_results.X_ref = X_ref;
MPC_results.X1 = X(1:12,:,1);
MPC_results.X2 = X(1:12,:,2);
MPC_results.X3 = X(1:12,:,3);
MPC_results.X4 = X(1:12,:,4);
MPC_results.U1 = U(:,:,1);
MPC_results.U2 = U(:,:,2);
MPC_results.U3 = U(:,:,3);
MPC_results.U4 = U(:,:,4);
MPC_results.X1_ref = X_ref(:,:,1);
MPC_results.X2_ref = X_ref(:,:,2);
MPC_results.X3_ref = X_ref(:,:,3);
MPC_results.X4_ref = X_ref(:,:,4);

MPC_results.error.pos = pos_error;
MPC_results.error.att = att_error;
MPC_results.error.all = cat(1, pos_error, att_error);

MPC_results.U_pos = U_pos;
MPC_results.simtime = MPC_sim_time;
MPC_results.tracking_error = tracking_error_xyz';
MPC_results.tracking_error_norm = tracking_error_xyz_norm;
MPC_results.formation_error = formation_error;
MPC_results.formation_error_norm = formation_error_norm';
MPC_results.formation_error_int = formation_error_int;

MPC_results.r_smooth1 = squeeze(X_ref(1:6,:,1));
MPC_results.r_smooth2 = squeeze(X_ref(1:6,:,2));
MPC_results.r_smooth3 = squeeze(X_ref(1:6,:,3));
MPC_results.r_smooth4 = squeeze(X_ref(1:6,:,4));

MPC_results.time = T;
MPC_results.offsets = offsets;
end
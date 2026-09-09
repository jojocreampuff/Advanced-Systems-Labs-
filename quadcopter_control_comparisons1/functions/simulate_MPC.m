function MPC_results = simulate_MPC(position, attitude, global_parameters, ref, IC, sim_params)
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
[~, K_att_lqr] = get_LQR_gains(10*Q_pos,10*R_pos, Q_att, R_att, Ix, Iy, Iz);
%% MPC stuff
% MPC parameters
T_pos_horizon = .3;  % seconds
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
X_ref(1:6,:) = r_smooth;
tic

for i = 1:length(T)
    %% add payload uncertianty 10 times during simulation
    if payload_uncert_logic  == 1 && mod(i,500) == 0
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

    %% MPC Position
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);

    if i+Hp <= length(X_ref(1,:))
        pos_ref_horizon = X_ref(1:6,i:i+Hp);
    else
        pos_ref_horizon = pos_ref_horizon;
    end

    [pos_control_guess] = a_MPC_Pos_QP(pos_ref_horizon, X(1:6, i), pos_control_guess, Hp, Q_pos, R_pos, dt);
    pos_control_guess = pos_control_guess + [0;0;g];
    U_pos(:,i) = pos_control_guess(:,1);

    %% our system solver
    [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i), -U_pos(3,i), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

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


function Discounted_Single_Critic_results = simulate_Discounted_Single_Critic(position, attitude, global_parameters, ref, IC, sim_params)
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

%% Discounted single critic stuff
load("z_discounted_single_critic_att.mat", "W", "R")
W_att = W;
R_att_DSC = R;

load("z_discounted_single_critic_pos.mat", "W","R")
W_pos = W;
R_pos_DSC = R;

% f_pos = @(x) [x(4); x(5); x(6); 0; 0; 0];
g_pos = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
g_att = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];

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

L = 0.2; % l in cm
max_Ft = 2*m*g;
Tmax = max_Ft/4;
Max_torque = Tmax*L*2;

u1_max = Max_torque; u2_max = Max_torque; u3_max = 0.7*Max_torque;
Umax = [u1_max; u2_max; u3_max];

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
    %% add payload uncertianty 10 times during simulation
    if payload_uncert_logic == 1 && mod(i,500) == 0
        m = inertia_params(1) + payload_uncert(1)*randn;
        Ix = inertia_params(2) + payload_uncert(2)*randn;
        Iy = inertia_params(3) + payload_uncert(3)*randn;
        Iz = inertia_params(4) + payload_uncert(4)*randn;
        g_att = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];
    end
    %% add sensor noise
    X(:,i) = add_state_noise(X(:,i), state_noise);
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

    U_pos(:,i) = -0.5*R_pos_DSC^-1*g_pos'*a_Discounted_Single_Critic_Grad_phi_pos(pos_error(:,i))'*W_pos' - [0; 0; g];
    
    [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i), r_yaw(i), m);
    
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    % AC controller used to track angles - error regulation and optimal control equation
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = -0.5*R_att_DSC^-1*g_att'*a_Discounted_Single_Critic_Grad_phi_pos(att_error(:,i))'*W_att';

    U(:,i) = [ft; torques(:,i)];

    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
    end

    U(:,i) = add_control_noise(U(:,i), control_noise);
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

Discounted_Single_Critic_sim_time = toc;
%% save all useful info in a struck
Discounted_Single_Critic_results.X = X;
Discounted_Single_Critic_results.U = U;
Discounted_Single_Critic_results.X_ref = X_ref;
Discounted_Single_Critic_results.error = [pos_error;att_error];
Discounted_Single_Critic_results.inscost = [instant_cost_pos; instant_cost_att];
Discounted_Single_Critic_results.cumcost = [cumulative_cost_pos; cumulative_cost_att];
Discounted_Single_Critic_c_cost_total = sum(Discounted_Single_Critic_results.cumcost, 1);
Discounted_Single_Critic_results.Discounted_Single_Critic_c_cost_total = Discounted_Single_Critic_c_cost_total;
Discounted_Single_Critic_results.simtime = Discounted_Single_Critic_sim_time;
Discounted_Single_Critic_error = Discounted_Single_Critic_results.error;
Discounted_Single_Critic_tracking_error = (Discounted_Single_Critic_error(1,:).^2 + Discounted_Single_Critic_error(2,:).^2 + Discounted_Single_Critic_error(3,:).^2).^0.5;
Discounted_Single_Critic_results.Discounted_Single_Critic_tracking_error = Discounted_Single_Critic_tracking_error;
Discounted_Single_Critic_results.Discounted_Single_Critic_tracking_error_norm = dt*sum(Discounted_Single_Critic_tracking_error,2);

[tot_cost, track_cost, control_cost] = calculate_cost(Discounted_Single_Critic_results.error,Discounted_Single_Critic_results.U,dt);
Discounted_Single_Critic_results.all_cost = [tot_cost, track_cost, control_cost];

end

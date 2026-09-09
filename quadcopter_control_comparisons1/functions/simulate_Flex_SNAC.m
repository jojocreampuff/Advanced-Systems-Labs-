function Flex_SNAC_results = simulate_Flex_SNAC(position, attitude, global_parameters, ref, IC, sim_params)
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

%% load flexible SNAC tuuuf
 
load("z_flex_att_don_dim.mat","Attitude_R","Attitude_W")
Attitude_Rflex = Attitude_R;
load("AttitudeWTroubleshoot2.mat")
Attitude_Wflex = Attitude_W;
%% offline SNAC shit
load('z_offline_SNAC_pos_data.mat','Position_W','Position_R',"Position_g","Position_G")
load('z_offline_SNAC_att_data.mat', "Attitude_W", "Attitude_R","max_states")
x1_max = max_states(1);
x2_max = max_states(2);
x3_max = max_states(3);
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
Attitude_G_bar = @(x_bar) Attitude_g_bar * dt;
% Position_G = [0 0 0; 0 0 0; 0 0 0; -1 0 0; 0 -1 0; 0 0 -1];
Attitude_G = [0 0 0;0 0 0;0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];

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
ft1 = zeros(1,N);   
r_initial       = zeros(3,N); % original trajectory
instant_cost_pos = zeros(1, N); % Instantaneous cost
cumulative_cost_pos = zeros(1, N); % Integrated cost
instant_cost_att = zeros(1, N); % Instantaneous cost
cumulative_cost_att = zeros(1, N); % Integrated cost
uRu_att = zeros(1, N); % control input
ft_log = zeros(1, N);
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

X(:,1) = IC;

Guided_Time = 3;
%N1 = round(Guided_Time/dt) + 1; %why is +1 needed
N1 = round(Guided_Time/dt);
N1 = min(max(N1,1), N);          
% look_ahead_t=length(Attitude_W(1,1,:));
% time = 0:dt:(look_ahead_t/1000+tf-dt); % see if needed
temp=1; % use for flexible path

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
for i = 1:N1
    %% add payload uncertianty 10 times during simulation
    if payload_uncert_logic == 1 && mod(i,500) == 0
        m = inertia_params(1) + payload_uncert(1)*randn;
        Ix = inertia_params(2) + payload_uncert(2)*randn;
        Iy = inertia_params(3) + payload_uncert(3)*randn;
        Iz = inertia_params(4) + payload_uncert(4)*randn;
        Attitude_g_bar = [0 0 0; 0 0 0; 0 0 0; u1_max/(Ix*x4_max) 0 0; 0 u2_max/(Iy*x5_max) 0; 0 0 u3_max/(Iz*x6_max)];
        Attitude_G_bar = @(x_bar) Attitude_g_bar * dt;
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

    U_pos(:,i) =  -Position_R^-1 * Position_G(pos_error(:,i))' * Position_W' * a_Offline_SNAC_phi_pos(pos_error(:,i));

    [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i), -U_pos(3,i), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    % SNAC controller used to track angles - error regulation and optimal control equation
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);

    torques(:,i) = -Attitude_R^-1 * Attitude_G_bar(att_error(:,i)./max_states)' * Attitude_W' * a_Offline_SNAC_phi_att(att_error(:,i)./max_states);
    torques(:,i) = torques(:,i).*Umax;

    U(:,i) = [ft; torques(:,i)];

    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
    end

    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end
    uRu_att(i) = U(2:4,i)' * R_att * U(2:4,i);
    ft_log(i) = U(1,i);
    %% Update Nonlinear Dynamics using Euler integration
    f_x = Full_f(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g(X(:, i), m,Ix,Iy,Iz);
    
    %% State update using Euler integration
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end

    instant_cost_pos(i) = dt * (pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos(:,i)' * R_pos * U_pos(:,i));
    instant_cost_att(i) = dt * (att_error(:,i)' * Q_att * att_error(:,i) + torques(:,i)' * R_att * torques(:,i));
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
   end
end

K = 10;             % Position controller update every K steps, can change/tune

for i = N1:N-1 % flex attitude starts
    %% add payload uncertianty 10 times during simulation
    if payload_uncert_logic && mod(i,500) == 0
        m = inertia_params(1) + payload_uncert(1)*randn;
        Ix = inertia_params(2) + payload_uncert(2)*randn;
        Iy = inertia_params(3) + payload_uncert(3)*randn;
        Iz = inertia_params(4) + payload_uncert(4)*randn;
        % make this update g_att
        g_att = [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];
    end
    %% add sensor noise
    if state_noise ~= 0
        X(:,i) = add_state_noise(X(:,i), state_noise);
    end
    %% add wind disturbence
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), rho, Cd, A, r, m, Ix, Iy, Iz);

        axyz_uncert     = axyz_uncert(:);        % 3x1
        alphaxyz_uncert = alphaxyz_uncert(:);    % 3x1

        X(:,i) = X(:,i) + dt * [ ...
            0;0;0; ...
            axyz_uncert; ...
            0;0;0; ...
            alphaxyz_uncert ];
    end
    X_ref(1:6,i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6,i) - X_ref(1:6,i);
  do_update = (mod(i - N1, K) == 0);
%    do_update = true;
   if do_update
       idx = min(i + K, size(r_smooth,2));
       r_future = r_smooth(:, idx);
       U_pos(:,i) = -Position_R^-1 * Position_G(X(1:6,i) - r_future)'* Position_W' * a_Offline_SNAC_phi_pos(X(1:6,i) - r_future);
       % U_pos(:,i) = max(min( U_pos(:,i), 20), -20);%testing, remove later,limit U_pos 

       [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i),-U_pos(3,i), r_yaw(i), m);
       ft1(i)=  ft;
       angles(:,i) = [r_pitch; r_roll; r_yaw(i)];

       % ft_angles = system_solver(U_pos(:,i), r_yaw(i), m);
       % ft1(i) = ft_angles(1);
       % angles(:,i) = [ft_angles(2); ft_angles(3); r_yaw(i)];

       %X_ref(7:12,i) = [angles(:,i); angle_rate_solver(angles,i,dt)];
        X_ref(7:12,i) = [angles(:,i); 0; 0; 0]; % red flag
   else
       % Hold last control
       U_pos(:,i) = U_pos(:,i-1);
       angles(:,i) = angles(:,i-1);
       ft1(i)=ft1(i-1);
       X_ref(7:12,i) = X_ref(7:12,i-1);

   end
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = -Attitude_Rflex^-1 * Attitude_G' * Attitude_Wflex(:,:,temp)' * Basis_Func_new(att_error(:,i));
    U(:,i) = [ft1(i); torques(:,i)];
    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
    end

    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end
    uRu_att(i) = U(2:4,i)' * R_att * U(2:4,i);
    ft_log(i) = U(1,i);

    %% Update Nonlinear Dynamics using Euler integration
    f_x = Full_f(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g(X(:, i), m,Ix,Iy,Iz);
    %% State update using Euler integration
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end
    temp = temp + 1;
    if temp > size(Attitude_Wflex, 3)
        temp = 1;
    end
    if any(~isfinite(X(:,i+1)))
        fprintf("Bad X at i=%d t=%.4f\n", i, i*dt);
        disp(X(:,i+1));
        break;
    end
    %X(1:6, i) = r_smooth(:,i);
    instant_cost_pos(i) = dt * (pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos(:,i)' * R_pos * U_pos(:,i));
    instant_cost_att(i) = dt * (att_error(:,i)' * Q_att * att_error(:,i) + torques(:,i)' * R_att * torques(:,i));
    cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
    cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
end




Flex_SNAC_sim_time = toc;
%% save all useful info in a struck
Flex_SNAC_results.X = X; % 12 x N
Flex_SNAC_results.U = U; % 4 X N
Flex_SNAC_results.X_ref = X_ref;
Flex_SNAC_results.error = [pos_error;att_error]; % 12 x N
Flex_SNAC_results.inscost = [instant_cost_pos; instant_cost_att]; % 2 X N
%should fix cost calcs
Flex_SNAC_results.cumcost = [cumsum(instant_cost_pos); cumsum(instant_cost_att)];
%Flex_SNAC_results.cumcost = [cumulative_cost_pos; cumulative_cost_att]; % 2 X N
Flex_SNAC_c_cost_total = sum(Flex_SNAC_results.cumcost, 1); % 1 x N
Flex_SNAC_results.Flex_SNAC_c_cost_total = Flex_SNAC_c_cost_total; % 1 x N
Flex_SNAC_results.simtime = Flex_SNAC_sim_time;
Flex_SNAC_error = Flex_SNAC_results.error;
Flex_SNAC_results.guided_time = Guided_Time;
Flex_SNAC_results.switch_index = N1;
Flex_SNAC_tracking_error = (Flex_SNAC_error(1,:).^2 + Flex_SNAC_error(2,:).^2 + Flex_SNAC_error(3,:).^2).^0.5;
Flex_SNAC_results.Flex_SNAC_tracking_error = Flex_SNAC_tracking_error; % 1 x N
Flex_SNAC_results.Flex_SNAC_tracking_error_norm = dt*sum(Flex_SNAC_tracking_error,2); % 1x1
Flex_SNAC_results.uRu_att = uRu_att;
Flex_SNAC_results.ft = ft_log;
[tot_cost, track_cost, control_cost] = calculate_cost(Flex_SNAC_results.error,Flex_SNAC_results.U,dt);
Flex_SNAC_results.all_cost = [tot_cost, track_cost, control_cost];
end

% function pqr = deriv(angles, i, dt)
% if i == 1
%     pqr = ((angles(:, i) - 0)/dt);
% elseif i > 1
%     pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
% end
% % Limit pqr to be between -5 and 5
% pqr(pqr > 5) = 5;
% pqr(pqr < -5) = -5;
% end

function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end

function uvw = discrete_deriv_simplified(x2,x1,dt)
    uvw(:, 1) = (x2 - x1)/dt;
end
% 
% 
% 
function out = system_solver(uxyz,r_psi,m)
eqns = @(vars) [uxyz(1) - vars(1)/m.*(sin(vars(2)).*sin(r_psi) + cos(vars(2)).*cos(r_psi).*sin(vars(3)));...
                uxyz(2) - vars(1)/m.*(cos(vars(2)).*sin(r_psi).*sin(vars(3)) - cos(r_psi).*sin(vars(2)));...
                uxyz(3) - vars(1)/m.*(cos(vars(2)).*cos(vars(3)))];

% initial guess for the solution
x0 = [1; pi/4; pi/6];

% solve the system of equations
options = optimset('Display','off');
sol = fsolve(eqns, x0,options);

out(1) = sol(1); %ft
out(2) = sol(2); %r_phi
out(3) = sol(3); %r_theta

end
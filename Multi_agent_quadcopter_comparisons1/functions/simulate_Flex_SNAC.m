function Flex_SNAC_results = simulate_Flex_SNAC(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
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

%% load flexible attitude SNAC stuuuf
load("z_flex_att_fixed.mat","Attitude_R","Attitude_W")
Attitude_Rflex = Attitude_R;
Attitude_Wflex = Attitude_W;
Attitude_G = [0 0 0;0 0 0;0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];
K = 10;             % Position controller update every K steps, can change/tune
%% offline positon SNAC
load('z_offline_SNAC_pos_data_MORE_R.mat','Position_W','Position_R',"Position_g","Position_G")

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

X(:,1) = IC;
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

for i = 1:length(T)
    %% add sensor noise
    if state_noise ~= 0
        X(:,i) = add_state_noise(X(:,i), state_noise);
    end
    %% add wind disturbence
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), m, Ix, Iy, Iz);

        axyz_uncert     = axyz_uncert(:);        % 3x1
        alphaxyz_uncert = alphaxyz_uncert(:);    % 3x1

        X(:,i) = X(:,i) + dt * [ 0;0;0;  axyz_uncert;  0;0;0; alphaxyz_uncert ];
    end

    X_ref(1:6,i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6,i) - X_ref(1:6,i);
    if mod(i-1,10) == 0
        %% remove the future look ahead
       % idx = min(i + 10, size(r_smooth,2));
       r_future = r_smooth(:, i);
       %% SNAC Position CONTROLLER
       U_pos(:,i) = -Position_R^-1 * Position_G(X(1:6,i) - r_future)'* Position_W' * a_Offline_SNAC_phi_pos(X(1:6,i) - r_future);
       U_pos(:,i) = max(min( U_pos(:,i), 20), -20);%testing, remove later,limit U_pos 
       [ft, r_pitch, r_roll] = system_solve(-U_pos(1,i), -U_pos(2,i),-U_pos(3,i), r_yaw(i), m);
       ft1(i)=  ft;
       angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
       X_ref(7:12,i) = [angles(:,i); angle_rate_solver(angles,i,dt)];
   else
       % Hold last control
       U_pos(:,i) = U_pos(:,i-1);
       angles(:,i) = angles(:,i-1);
       ft1(i)=ft1(i-1);
       X_ref(7:12,i) = X_ref(7:12,i-1);
   end

    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = -Attitude_Rflex^-1 * Attitude_G' * Attitude_Wflex(:,:,temp)' * a_phi_att_flex(att_error(:,i));
    U(:,i) = [ft1(i); torques(:,i)];
    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
    end
    temp = temp + 1;
    if temp > size(Attitude_Wflex,3)
        temp = 1;
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


Flex_SNAC_sim_time = toc;
%% save all useful info in a struck
Flex_SNAC_results.X = X; % 12 x N
Flex_SNAC_results.U = U; % 4 X N
Flex_SNAC_results.X_ref = X_ref;
Flex_SNAC_results.error = [pos_error;att_error]; % 12 x N
Flex_SNAC_results.simtime = Flex_SNAC_sim_time;
Flex_SNAC_error = Flex_SNAC_results.error;
Flex_SNAC_tracking_error = (Flex_SNAC_error(1,:).^2 + Flex_SNAC_error(2,:).^2 + Flex_SNAC_error(3,:).^2).^0.5;
Flex_SNAC_results.Flex_SNAC_tracking_error = Flex_SNAC_tracking_error; % 1 x N
Flex_SNAC_results.Flex_SNAC_tracking_error_norm = dt*sum(Flex_SNAC_tracking_error,2); % 1x1

end

function FL_results = simulate_FL(seed,position, attitude, global_parameters, ref, IC, sim_params, Wxyz)
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

%% Feedback linearization gains (choose as 2nd-order linear error dynamics)
% Position: e_ddot + Kd_pos*e_dot + Kp_pos*e = 0
Kp_pos = 1*diag([1 1 1]);     
Kd_pos = 1*diag([1 1 1]);    

% Attitude outer->rate shaping (same role as Lambda_att in your SMC version)
Lambda_att = diag([8 8 4]); 

% Rate loop (feedback linearization with PD on omega tracking)
Kp_rate = diag([8 8 4]);
Kd_rate = diag([0 0 0]);

f_att_rate = @(ohm) [((Iy - Iz) / Ix * ohm(2) * ohm(3)); 
                    ((Iz - Ix) / Iy * ohm(1) * ohm(3)); 
                    ((Ix - Iy) / Iz * ohm(1) * ohm(2))];

g_att_rate = [1/Ix 0 0; 
              0 1/Iy 0; 
              0 0 1/Iz];

Trans = @(angles) [ 1,  sin(angles(1))*tan(angles(2)),  cos(angles(1))*tan(angles(2));
                    0,  cos(angles(1)),            -sin(angles(1));
                    0,  sin(angles(1))/cos(angles(2)),  cos(angles(1))/cos(angles(2))];

ohm_d = zeros(3,N);
omega_d_dot_filt = zeros(3,1);

% Preallocating variables
X               = zeros(12,N);
U               = zeros(4, length(T));
X_ref           = zeros(12, length(T));
angles          = zeros(3,length(T));
r_yaw           = global_parameters.r_yaw;
U_pos            = zeros(3,N);
torques         = zeros(3,N);
att_error       = zeros(6,N);
pos_error       = zeros(6,N);
r_initial       = zeros(3,N-1);

X(:,1) = IC;

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

r_xyz_ddot = discrete_deriv(r_smooth(4:6,:),dt);

tic
for i = 1:length(T)
    %% add sensor noise
    if state_noise ~= 0
        X(:,i) = add_state_noise(X(:,i), state_noise);
    end

    %% add wind disturbance
    if wind_logic == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), m, Ix, Iy, Iz);
        X(:,i) = X(:,i)+ dt*[0;0;0;
                             axyz_uncert;
                             0;0;0;
                             alphaxyz_uncert];
    end

    %% Outer-loop: feedback-linearized position control (virtual acceleration command)
    % Reference pos/vel/acc
    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);

    %% new rate positon loop
    if mod(i-1,10) == 0
        %% remove the future look ahead
        % idx = min(i + 10, N);
        r_future = r_smooth(:, i);
        r_xzy_ddot_future = r_xyz_ddot(:,i);
        e_pos     = X(1:3, i) - r_future(1:3);
        e_pos_dot = X(4:6, i) - r_future(4:6);
    
        a_cmd = r_xzy_ddot_future - Kd_pos*e_pos_dot - Kp_pos*e_pos;
    
        U_pos(:,i) = [a_cmd(1); a_cmd(2); a_cmd(3)];
    
        a_thrust_cmd = U_pos(:,i) - [0;0;g];
    
        if saturation_logic == 1
            a_thrust_max = 2*g; 
    
            if norm(a_thrust_cmd) > a_thrust_max
                a_thrust_cmd = a_thrust_cmd * (a_thrust_max / norm(a_thrust_cmd));
            end
        end
    
        U_pos(:,i) = a_thrust_cmd;
        [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i), r_yaw(i), m);
        angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
        X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];
    else
        U_pos(:,i) = U_pos(:,i-1);
        X_ref(7:12,i) = X_ref(7:12,i-1);
        angles(:,i) = angles(:,i-1);
    end
    
    %% Map acceleration command -> thrust + desired roll/pitch using your solver
    % [ft, r_pitch, r_roll] = system_solve(U_pos(1,i), U_pos(2,i), U_pos(3,i), r_yaw(i), m);
    % angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    % X_ref(7:12, i) = [angles(:,i); angle_rate_solver(angles,i,dt)];

    %% Inner-loop: feedback linearization on rate dynamics (computed torque)
    att_error(:,i) = X(7:12,i) - X_ref(7:12, i);

    % Commanded body rate from Euler error shaping:
    % ohm_d = Trans(eta)^{-1} (eta_ref_dot - Lambda*e_eta)
    ohm_d(:,i) = Trans(X(7:9,i)) \ (X_ref(10:12, i) - Lambda_att * att_error(1:3,i));

    %% first order filter
    omega_d_dot_raw = angle_rate_solver(ohm_d,i,dt); % assumes this returns derivative of a vector input
    alpha = 0.9;
    omega_d_dot = alpha * omega_d_dot_filt + (1-alpha) * omega_d_dot_raw;
    omega_d_dot_filt = omega_d_dot;

    % Rate tracking error
    e_omega = X(10:12,i) - ohm_d(:,i);

    % Feedback linearization / computed torque:
    % omega_dot = f(omega) + g*tau
    % choose tau to make: e_omega_dot = -Kp_rate*e_omega   (or add -Kd_rate*e_omega_dot if you estimate it)
    v_rate = omega_d_dot - Kp_rate*e_omega;  % desired omega_dot

    % tau = g^{-1} ( v_rate - f(omega) )
    torques(:,i) = g_att_rate^-1 * ( v_rate - f_att_rate(X(10:12,i)) );
    

    %% Assemble input
    U(:,i) = [ft; torques(:,i)];

    if saturation_logic == 1
        U(:,i) = saturate_controls(U(:,i));
    end

    if control_noise ~= 0
        U(:,i) = add_control_noise(U(:,i), control_noise);
    end

    %% Integrate dynamics
    f_x = Full_f(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g(X(:, i), m,Ix,Iy,Iz);

    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end
end

FL_sim_time = toc;
FL_results.X = X;
FL_results.U = U;
FL_results.X_ref = X_ref;
FL_results.error = [pos_error;att_error];
FL_results.simtime = FL_sim_time;
FL_error = FL_results.error;
FL_tracking_error = (FL_error(1,:).^2 + FL_error(2,:).^2 + FL_error(3,:).^2).^0.5;
FL_results.FL_tracking_error = FL_tracking_error;            % keep field name
FL_results.FL_tracking_error_norm = dt*sum(FL_tracking_error,2);

% [tot_cost, track_cost, control_cost] = calculate_cost(FL_results.error,FL_results.U,dt);
% FL_results.all_cost = [tot_cost, track_cost, control_cost];

end

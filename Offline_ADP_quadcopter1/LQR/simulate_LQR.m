function results = simulate_LQR(Position, Attitude, global_parameters, ref, IC, control_noise, state_noise, saturation_logic, wind_logic)
Position_R = Position.Position_R;
Position_Q = Position.Position_Q;

Attitude_R = Attitude.Attitude_R;
Attitude_Q = Attitude.Attitude_Q;

dt      = global_parameters.dt;
g       = global_parameters.g;
tf      = global_parameters.tf;
m       = global_parameters.m;
Ix      = global_parameters.Ix;
Iy      = global_parameters.Iy;
Iz      = global_parameters.Iz;

T = 0:dt:tf-dt;
N = length(T);

% Preallocating variables
u               = zeros(4,N-1); % quadcopter controls [ft tx ty tz]
u_noise         = zeros(4,N-1); % noisy quadcopter controls
uxyz            = zeros(3,N);   % output of position SNAC
torques         = zeros(3,N);   % output of attitude SNAC
ft              = zeros(1,N);   % thrust
angles          = zeros(3,N);   % angles
angles_ref      = zeros(6,N-1); % angles and angular velocities
Att_error  = zeros(6,N);   % error 
Pos_error = zeros(6,N);
r_initial       = zeros(3,N-1); % original trajectory
r_phi           = zeros(1,N);   % phi trajectory
r_the           = zeros(1,N);   % theta trajectory
r_psi           = ones(1,N);   % psi trajectory
instant_cost_pos = zeros(1, N); % Instantaneous cost
cumulative_cost_pos = zeros(1, N); % Integrated cost
instant_cost_att = zeros(1, N); % Instantaneous cost
cumulative_cost_att = zeros(1, N); % Integrated cost

Full_F = @(x,g,Ix,Iy,Iz) x + dt * Full_f(x,g,Ix,Iy,Iz); % discretized drift dynamics
Full_G = @(x,m,Ix,Iy,Iz) dt * Full_g(x,m,Ix,Iy,Iz);           % discretized control dynamics

x(:,1) = IC; % defining inital x as the initial condtion (IC)

% determining modified reference trajectory based on original trajectory
for i = 1:length(T)
    r_initial(:,i) = ref(T(i));
end

smooth_r_position = smooth(r_initial, x(1:3)', x(4:6)', T);
% smooth_r_position = r_initial;
r_smooth = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];

%% disturbance and uncertainty
sensor_uncert = [0.02, 0.02, 0.05, 0.05, 0.05, 0.05, 0.15, 0.15, 0.5, 0.005, 0.005, 0.005]';
actuator_uncert = [0.05*Tmax, 0.005*Max_torque, 0.005*Max_torque, 0.01*Max_torque]';

for i = 1:length(T)
    % SNAC controller used to track trajectory - optimal control equation
    Pos_error(:,i) = x(1:6,i) - r_smooth(:, i);

    uxyz = -Position_R^-1 * Position_G(Pos_error(:,i))' * Position_W(:,:)' * Basis_Func_pos(Pos_error(:,i));
   
    [ft(i), r_phi(i), r_the(i)] = borna_sys_solve(-uxyz(1,i), -uxyz(2,i), -uxyz(3,i), r_psi(i), m);
    
    angles(:,i) = [r_phi(i); r_the(i); r_psi(i)];
    angles_ref(:,i) = [angles(:,i); deriv(angles,i,dt)];

    % SNAC controller used to track angles - error regulation and optimal control equation
    Att_error(:,i) = x(7:12,i) - angles_ref(:,i);
    torques(:,i) = -Attitude_R^-1 * Attitude_G_bar(Att_error(:,i)./max_att_states)' * Attitude_W' * Basis_Func_84(Att_error(:,i)./max_att_states);
    torques(:,i) = torques(:,i).*Umax; %% turn u_bar into u, unit is now Nm

    u(:,i) = [ft(i); torques(:, i)];
    %% add noise to states
    % wind disturbance
    x(:,i) = add_noise(x(:,i), wind_dist, noise(1));
    % sensor uncertainty
    x(:,i) = x(:,i) + noise(2)*sensor_uncert .* randn(size(sensor_uncert)); % Gaussian noise
    % actuator uncertainty
    u_noise(:,i) = add_noise(u(:,i),actuator_uncert, noise(3)); % add randomness

    if u_noise(1,i) < 0 
        u_noise(1,i) = 0;
    end

    % Passing controls though discretized drone dynamics
    x(:, i+1) = Full_F(x(:,i),grav,Ix,Iy,Iz) + Full_G(x(:,i),m,Ix,Iy,Iz) * u_noise(:,i);

    %% cost plotting
    instant_cost_pos(i) = Pos_error(:,i)' * Position_Q * Pos_error(:,i) + uxyz(:,i)' * Position_R * uxyz(:,i);
    instant_cost_att(i) = Att_error(:,i)' * Attitude_Q * Att_error(:,i) + torques(:,i)' * Attitude_R * torques(:,i);
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i) * dt;
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i) * dt;
   end
end

results.x = x;
results.u = u_noise;
results.r_smooth = r_smooth;
results.r_initial = r_initial;
results.angles_ref = angles_ref;
results.time = time;
results.uxyz = uxyz;
results.Pos_error = Pos_error;
results.Att_error = Att_error;
results.PWM_channels = PWM_channels;
results.each_motor_thrust = each_motor_thrust;
results.instant_cost_pos = instant_cost_pos;
results.instant_cost_att = instant_cost_att;
results.cumulative_cost_pos = cumulative_cost_pos;
results.cumulative_cost_att = cumulative_cost_att;
function results = simulate(Position, Attitude, parameters, reference, IC, noise)

Position_W = Position.Position_W;
Position_G = Position.Position_G;
Position_R = Position.Position_R;
Position_Q = Position.Position_Q;

Attitude_W = Attitude.Attitude_W;
Attitude_G = Attitude.Attitude_G;
Attitude_R = Attitude.Attitude_R;
Attitude_Q = Attitude.Attitude_Q;
Attitude_G_bar = Attitude.Attitude_G_bar;
Attitude_F_bar = Attitude.Attitude_F_bar;
max_att_states = Attitude.Attitude_max_states;

dt      = parameters.dt;
grav    = parameters.grav;
t_f     = parameters.t_f;
m       = parameters.m;
Ix      = parameters.Ix;
Iy      = parameters.Iy;
Iz      = parameters.Iz;

N = t_f/dt;

% Preallocating variables
u               = zeros(4,N-1); % quadcopter controls [ft tx ty tz]
u_noise         = zeros(4,N-1); % noisy quadcopter controls
PWM_channels    = zeros(4,N-1);
each_motor_thrust = zeros(4,N-1);
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
% r_psi           = [ linspace(-pi, pi/2, N/4), linspace(pi, -pi/2, N/4), linspace(-pi, pi/2, N/4), linspace(pi, -pi/2, N/4) ];    % saftey limit threshold on desired pitch and roll
% r_psi           = [0*ones(1,N/10), pi/4*ones(1,N/10), pi/4*ones(1,N/10), pi/2*ones(1,N/10), pi/2*ones(1,N/10), pi/4*ones(1,N/10), pi/4*ones(1,N/10), pi/2*ones(1,N/10), pi/2*ones(1,N/10), pi/4*ones(1,N/10)];
instant_cost_pos = zeros(1, N); % Instantaneous cost
cumulative_cost_pos = zeros(1, N); % Integrated cost
instant_cost_att = zeros(1, N); % Instantaneous cost
cumulative_cost_att = zeros(1, N); % Integrated cost

Full_F = @(x,grav,Ix,Iy,Iz) x + dt * Full_f_225(x,grav,Ix,Iy,Iz); % discretized drift dynamics
Full_G = @(x,m,Ix,Iy,Iz) dt * Full_g_225(x,m,Ix,Iy,Iz);           % discretized control dynamics

x(:,1) = IC; % defining inital x as the initial condtion (IC)

time = 0:dt:t_f-dt;

% determining modified reference trajectory based on original trajectory
for i = 1:length(time)
    r_initial(:,i) = reference(time(i));
end

% smooth_r_position = smooth(r_initial, x(1:3)', x(4:6)', time);
smooth_r_position = r_initial;
r_smooth = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];

r = 0.2;
drag = 0.02; % (drag coef)
max_Ft = 2*m*grav;
Tmin = 0;
Tmax = max_Ft/4;
Max_torque = Tmax*r*2;

%% disturbance and uncertainty
max_wind_force = 5; % N
max_wind_torque = max_wind_force*r;
wind_dist =    [ 0    0    0    max_wind_force/m    max_wind_force/m    max_wind_force/m    0    0    0    max_wind_torque/Ix    max_wind_torque/Iy    max_wind_torque/Iz]';
sensor_uncert = [0.02, 0.02, 0.05, 0.05, 0.05, 0.05, 0.15, 0.15, 0.5, 0.005, 0.005, 0.005]';
% g_noise = sensor_uncert .* randn(size(sigma)); % Gaussian noise
actuator_uncert = [0.05*Tmax, 0.005*Max_torque, 0.005*Max_torque, 0.01*Max_torque]';

% denormalization values
u1_max = Max_torque; u2_max = Max_torque; u3_max = Max_torque;
Umax = [u1_max; u2_max; u3_max];



for i = 1:N-1

    % SNAC controller used to track trajectory - optimal control equation
    Pos_error(:,i) = x(1:6,i) - r_smooth(:, i);

    uxyz(:,i) = -Position_R^-1 * Position_G(Pos_error(:,i))' * Position_W(:,:)' * Basis_Func_pos(Pos_error(:,i));
   
    [ft(i), r_phi(i), r_the(i)] = borna_sys_solve(-uxyz(1,i), -uxyz(2,i), -uxyz(3,i), r_psi(i), m);
    
    angles(:,i) = [r_phi(i); r_the(i); r_psi(i)];
    angles_ref(:,i) = [angles(:,i); deriv(angles,i,dt)];

    % SNAC controller used to track angles - error regulation and optimal control equation
    Att_error(:,i) = x(7:12,i) - angles_ref(:,i);
    torques(:,i) = -Attitude_R^-1 * Attitude_G_bar(Att_error(:,i)./max_att_states)' * Attitude_W' * Basis_Func_84(Att_error(:,i)./max_att_states);
    torques(:,i) = torques(:,i).*Umax; %% turn u_bar into u, unit is now Nm

    % % saturate the torques to the max torque the drone can produce
    % for p = 1:3
    %     if torques(p,i) > Max_torque
    %         torques(p,i) = Max_torque;
    %     elseif torques(p,i) < -Max_torque
    %         torques(p,i) = -Max_torque;
    %     end
    % end
    % % solve for the thrust of each motor given those torques    
    % T1 = (ft(i) - torques(1,i)/(2*r) - torques(2,i)/(2*r) + torques(3,i)/(4*drag)) / 4;
    % T2 = (ft(i) - torques(1,i)/(2*r) + torques(2,i)/(2*r) + torques(3,i)/(4*drag)) / 4;
    % T3 = (ft(i) + torques(1,i)/(2*r) - torques(2,i)/(2*r) - torques(3,i)/(4*drag)) / 4;
    % T4 = (ft(i) + torques(1,i)/(2*r) + torques(2,i)/(2*r) - torques(3,i)/(4*drag)) / 4;
    % each_motor_thrust(:,i) = [T1;T2;T3;T4];
    % 
    % % saturate each motor thrust to its bounded values
    % for j = 1:4
    %     if each_motor_thrust(j,i) > Tmax
    %         each_motor_thrust(j,i) = Tmax;
    %     elseif each_motor_thrust(j,i) < Tmin
    %         each_motor_thrust(j,i) = Tmin;
    %     end
    % end
    % 
    % % ignore this, this is solving for the PWM commands to send to a flight
    % % controller,they are values between 1000 and 2000 for min and max
    % % throttle
    % ch1 = 1000 + (((each_motor_thrust(1,i) - Tmin ) / (Tmax-Tmin)) * 1000 );
    % ch2 = 1000 + (((each_motor_thrust(2,i) - Tmin) /  (Tmax-Tmin)) * 1000 );
    % ch3 = 1000 + (((each_motor_thrust(3,i) - Tmin) /  (Tmax-Tmin)) * 1000 );
    % ch4 = 1000 + (((each_motor_thrust(4,i) - Tmin) /  (Tmax-Tmin)) * 1000 );
    % PWM_channels(:,i) = [ch1;ch2;ch3;ch4];
    % 
    % % Combining controls
    % % thrust cant be more then FT_max!
    % ft(1,i) = sum(each_motor_thrust(:,i));
    % 
    % if ft(1,i)>max_Ft
    %     ft(1,i) = max_Ft;
    % elseif ft(1,i)<0
    %     ft(1,i) = 0;
    % end

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

function pqr = deriv(angles, i, dt)
if i == 1
    pqr = ((angles(:, i) - 0)/dt);
elseif i > 1
    pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
end
% Limit pqr to be between -5 and 5
pqr(pqr > 1) = 1;
pqr(pqr < -1) = -1;
end

function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end


function [ft, pitch, roll] = borna_sys_solve(u1, u2, u3, psi, m) % u1 = ux , u2 = uy , u3 = uz - g 
    
    if psi <= 0.001 && psi >= -0.001
        psi = 0.001;
    end

    a = sin(psi);
    b = cos(psi);

    A = u1 / (u3);
    B = u2 / (u3);
    C = (a*A - b*B) / ( (a^2) + (b^2)) ;

    roll = atan( (B + b*C) / a );
    pitch = atan( C * cos(roll) );

    % saftey limit threshold on desired pitch and roll
    if pitch>pi/4
        pitch = pi/4;
    elseif pitch<-pi/4
        pitch = -pi/4;
    end

    if roll>pi/4
        roll = pi/4;
    elseif roll<-pi/4
        roll = -pi/4;
    end

    ft = ( m / (cos(pitch)*cos(roll)) ) * (-u3); 

end


end

function noisy_vector = add_noise(state_vector, mag, noise_percent)
    noise = (noise_percent) * mag .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end


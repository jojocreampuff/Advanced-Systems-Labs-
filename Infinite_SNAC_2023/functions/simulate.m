function results = simulate(Position, Attitude, parameters, reference, IC, noise)

Position_W = Position.Position_W;
Position_G = Position.Position_G;
Position_R = Position.Position_R;

Attitude_W = Attitude.Attitude_W;
Attitude_G = Attitude.Attitude_G;
Attitude_R = Attitude.Attitude_R;

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
uxyz            = zeros(3,N);   % output of position SNAC
ft_angles       = zeros(3,N);   % output of NN [ft phi theta] psi = 0
torques         = zeros(3,N);   % output of attitude SNAC
ft              = zeros(1,N);   % thrust
angles          = zeros(3,N);   % angles
angles_ref      = zeros(6,N-1); % angles and angular velocities
Attitude_error  = zeros(6,N);   % error 
r_initial       = zeros(3,N-1); % original trajectory
r_phi           = zeros(1,N);   % phi trajectory
r_the           = zeros(1,N);   % theta trajectory
r_psi           = zeros(1,N);   % psi trajectory

r_psi           = linspace(0, 4*pi, N);
% r_psi_prev = 0;
% for j= 1:N
% 
%      if mod(j,100) == 0
%         r_psi(j) = pi/24 + r_psi_prev ;
% 
%      else
%          r_psi(j) = r_psi_prev;
%      end
%         %  if r_psi(j) >= pi
%         %     r_psi(j) = -pi - 0.001;
%         %     r_psi_prev = 0;
%         % end
%      r_psi_prev = r_psi(j);
% 
% end

Full_F = @(x,grav,Ix,Iy,Iz) x + dt * Full_f_225(x,grav,Ix,Iy,Iz); % discretized drift dynamics
Full_G = @(x,m,Ix,Iy,Iz) dt * Full_g_225(x,m,Ix,Iy,Iz);           % discretized control dynamics

x = IC; % defining inital x as the initial condtion (IC)

time = 0:dt:t_f-dt;

% determining modified reference trajectory based on original trajectory
for i = 1:length(time)
    r_initial(:,i) = reference(time(i));
end
smooth_r_position = smooth(r_initial, x(1:3)', x(4:6)', time);
r_smooth = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];



for i = 1:N-1

    % SNAC controller used to track trajectory - optimal control equation
    uxyz(:,i) = -Position_R^-1 * Position_G(x(1:6,i) - r_smooth(:, i))' * Position_W(:,:)' * Basis_Func_pos(x(1:6,i) - r_smooth(:, i));
    
    % NN estimating ft and angles from ux, uy, and uz
    % ft_angles(:, i) = NN7(uxyz(1, i), uxyz(2, i), uxyz(3, i) );
    % ft(i) = ft_angles(1,i);
    % if ft(i) < 0
    %     ft(i) = 0;
    % end

    angles(:,i) = [ft_angles(2,i);ft_angles(3,i); 0];

%     Numerically solving for ft and angles from ux, uy, and uz
    % [ft(i), r_phi(i), r_the(i)] = system_solver(uxyz(:,i),0,m);

    [ft(i), r_phi(i), r_the(i)] = borna_sys_solve(-uxyz(1,i), -uxyz(2,i), -uxyz(3,i), r_psi(i), m);
    % [ft(i), r_phi(i), r_the(i)] = borna_sys_solve(-uxyz(1,i), -uxyz(2,i), -uxyz(3,i), x(9,i), m);
    
    angles(:,i) = [r_phi(i); r_the(i); r_psi(i)];
    angles_ref(:,i) = [angles(:,i); deriv(angles,i,dt)];

    % SNAC controller used to track angles - error regulation and optimal control equation
    Attitude_error(:,i) = x(7:12,i) - angles_ref(:,i);
    torques(:,i) = -Attitude_R^-1 * Attitude_G(Attitude_error(:,i))' * Attitude_W(:,:)' * Basis_Func_84(Attitude_error(:,i));
    
    torques(:,i) = saturation(torques(:,i));


    % Combining controls
    u(:,i) = [ft(1,i); torques(:, i)];
    u_noise(:,i) = u(:,i) .* (1 + noise.*(2*rand(size(u(:,i))) - 1)); % add randomness
    if u_noise(1,i) < 0 
        u_noise(1,i) = 0;
    end

    % Passing controls though discretized drone dynamics
    x(:, i+1) = Full_F(x(:,i),grav,Ix,Iy,Iz) + Full_G(x(:,i),m,Ix,Iy,Iz) * u_noise(:,i);

    % x(:,i+1) = x(:,i+1) .* (1 + noise.*(2*rand(size(x(:,i+1))) - 1)); % add randomness

end

results.x = x;
results.u = u_noise;
results.r_smooth = r_smooth;
results.r_initial = r_initial;
results.angles_ref = angles_ref;
results.time = time;

function pqr = deriv(angles, i, dt)
if i == 1
    pqr = ((angles(:, i) - 0)/dt);
elseif i > 1
    pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
end
% Limit pqr to be between -5 and 5
pqr(pqr > 5) = 5;
pqr(pqr < -5) = -5;
end

function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end

%     function [ft, phi, theta] = system_solver(uxyz,r_psi, m)
%     g = 9.81;
%         % F(x) = 0
%     eqns = @(vars) [uxyz(1) - vars(1)/m.*( sin(vars(2)).*sin(r_psi) + cos(vars(2)).*cos(r_psi).*sin(vars(3)) );...
%                     uxyz(2) - vars(1)/m.*( cos(vars(2)).*sin(r_psi).*sin(vars(3)) - cos(r_psi).*sin(vars(2)) );...
%                     uxyz(3) - vars(1)/m.*(cos(vars(2)).*cos(vars(3)))];
%         % vars(1) = ft;
%         % vars(2) = phi;
%         % vars(3) = theta;
% 
%         % initial guess for the solution
%         x0 = [1; pi/4; pi/6];
%         options = optimset('Display','off');
% 
%     % for k = 1:3
%     % 
%     %     out = fsolve(eqns, x0, options);
%     % 
%     %     x0 = out;
%     % end
% 
%     out = fsolve(eqns, x0, options);
% 
%     ft = out(1);
%     phi = out(2);
%     theta = out(3);
% 
% end

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

    % add a threshold (this is the max thrust of the combined actuators at
    % full throttle 
    if ft>72
        ft = 72;
    elseif ft<0
        ft = 0;
    end

end

    function torques = saturation(torques)

% based on max actuator output
        if torques(1) > 8
            torques(1) = 8;
        elseif torques(1) < -8
            torques(1) = -8;
        end

        if torques(2) > 8
            torques(2) = 8;
        elseif torques(2) < -8
            torques(2) = -8;
        end

        if torques(3) > 3
            torques(3) = 3;
        elseif torques(3) < -3
            torques(3) = -3;
        end

    end

end




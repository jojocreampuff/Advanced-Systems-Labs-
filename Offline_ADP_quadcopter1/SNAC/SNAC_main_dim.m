% SNAC MAIN 
clear; clc; close all;
rng(5,"twister")
g = 9.81;
tf = 50;
Ix = 0.3;   % Moment of inertia (kg*m^2)
Iy = 0.4;
Iz = 0.5;
m = 1;
% cost function parameters
%% Make sure that these are the same as in the snac training files
Position_Q = diag([1000,1000,1000,1000,1000,1000]);
Position_R = diag([1,1,1])*50; % SITL has too much control input
Q_pos = Position_Q;
R_pos = Position_R;

Attitude_Q = diag([10,10,10,1,1,1])*1000;
Attitude_R = diag([1,1,1])*50;
Q_att = Attitude_Q;
R_att = Attitude_R;

% Reference trajectory function
F_r = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);
            (1-exp(-0.01*t))*9.81*sin(0.2*t);
            -.1*t;];

%% load SNAC trained things
load('SNAC_pos_NEW.mat','Position_W','Position_Q','Position_R','Position_F','Position_G','dt','grav')
load('SNAC_att_dim.mat')
dt = 0.004;
%% **Simulation Initialization**
T = 0:dt:tf-dt;
N = length(T);
X = zeros(12, length(T));  
%% wind case IC
% X(:,1) = [5; 5; 0
% zeros(9,1)];
%% standard IC
X(:,1) = [5; 5; 0
-1.161992909093235
-5.108801409368473
0.790206167774050
0.1355
-0.542388992513929
pi/2
-0.0712468654804163
-0.00259254398073884
0.516408016763246];
%% failure mode IC
% X(:,1) = [-100; -100; -100
% 10
% 10
% -10
% -pi/4
% -pi/4
% pi
% pi/4
% -pi/4
% -pi/4];

U = zeros(4, length(T));  % Control input trajectory
X_ref = zeros(12, length(T));  % Reference trajectory
r_yaw = ones(1,N);
% cost plotting 
instant_cost_pos = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_pos = zeros(1, length(T)); % Integrated cost
instant_cost_att = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_att = zeros(1, length(T)); % Integrated cost

for j = 1:length(T)
    t = T(j);
    r_initial(:,j) = F_r(t);
end

smooth_r_position = smooth(r_initial, X(1:3), X(4:6), T);
r_smooth = [smooth_r_position; discrete_deriv(smooth_r_position,dt)];
r_smooth = [r_initial; discrete_deriv(r_initial,dt)];

%% Test 2: disturbance and uncertainty
add_wind = 0; % logical value
add_sensor_noise = 0; % percent noise 
add_actuator_noise = 0; % percent noise
add_saturation = 0; % logical value
load("wind_simulation.mat")
L = 0.2; % l in cm
drag = 0.02; % (drag coef)
max_Ft = 2*m*grav;
Tmin = 0;
Tmax = max_Ft/4;
Max_torque = Tmax*L*2;

sensor_uncert = [0.02, 0.02, 0.05, 0.05, 0.05, 0.05, 0.15, 0.15, 0.5, 0.005, 0.005, 0.005]';
actuator_uncert = [0.1*max_Ft, 0.01*Max_torque, 0.01*Max_torque, 0.02*Max_torque]';

u1_max = Max_torque; u2_max = Max_torque; u3_max = 0.7*Max_torque;
Umax = [u1_max; u2_max; u3_max];

tic
for i = 1:length(T)
    t = T(i);

    %% rebuttal 2 stuff
    % Position_R = Position_R + 0.1*[randn, 0, 0; 0, randn, 0;0, 0, randn];
    % Attitude_R = Attitude_R + 0.1*[randn, 0, 0; 0, randn, 0;0, 0, randn];
    % if i == .2*length(T)
    %     m =2;
    %     Ix = 0.5;
    %     Iy = 0.6;
    %     Iz = 0.7;
    % end
    % if i == .4*length(T)
    %     m =5;
    %     Ix = 0.5;
    %     Iy = 0.3;
    %     Iz = 0.4;
    % end
    % if i == .6*length(T)
    %     m =3;
    %     Ix = 0.6;
    %     Iy = 0.2;
    %     Iz = 0.3;
    % end
    % if i == .8*length(T)
    %     m =2;
    %     Ix = 0.2;
    %     Iy = 0.5;
    %     Iz = 0.7;
    % end
    %% add sensor noise
    X(:,i) = add_noise(X(:,i), sensor_uncert, add_sensor_noise);
    %% add wind disturbence
    if add_wind == 1
        [axyz_uncert, alphaxyz_uncert] = wind_f_m(Wxyz(:,i), X(:,i), rho, Cd, A, r, m, Ix, Iy, Iz);
        X(:,i) = X(:,i) + dt*[0;0;0;
                           axyz_uncert;
                           0;0;0;
                           alphaxyz_uncert];
    end
    %% position control
    X_ref(1:6, i) = r_smooth(:,i);
    pos_error(:,i) = X(1:6, i) - X_ref(1:6, i);
    U_pos =  -Position_R^-1 * Position_G(pos_error(:,i))' * Position_W' * Basis_Func_pos(pos_error(:,i));
    [ft, r_pitch, r_roll] = borna_sys_solve(-U_pos(1), -U_pos(2),-U_pos(3), r_yaw(i), m);
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    X_ref(7:12, i) = [angles(:,i); deriv(angles,i,dt)];

    %% att control (dynamics are non-dimensionalized)
    Att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques(:,i) = -Attitude_R^-1 * Attitude_G(Att_error(:,i))' * Attitude_W' * Basis_Func_84(Att_error(:,i));
    torques(:,i) = torques(:,i);

%% saturate the torques to the max torque the drone can produce
    if add_saturation ==1
        for p = 1:3
            if torques(p,i) > Max_torque
                torques(p,i) = Max_torque;
            elseif torques(p,i) < -Max_torque
                torques(p,i) = -Max_torque;
            end
        end
        % solve for the thrust of each motor given those torques    
        T1 = (ft - torques(1,i)/(2*L) - torques(2,i)/(2*L) + torques(3,i)/(4*drag)) / 4;
        T2 = (ft - torques(1,i)/(2*L) + torques(2,i)/(2*L) + torques(3,i)/(4*drag)) / 4;
        T3 = (ft + torques(1,i)/(2*L) - torques(2,i)/(2*L) - torques(3,i)/(4*drag)) / 4;
        T4 = (ft + torques(1,i)/(2*L) + torques(2,i)/(2*L) - torques(3,i)/(4*drag)) / 4;
        each_motor_thrust = [T1;T2;T3;T4];
    
        % saturate each motor thrust to its bounded values
        for j = 1:4
            if each_motor_thrust(j) > Tmax
                each_motor_thrust(j) = Tmax;
            elseif each_motor_thrust(j) < Tmin
                each_motor_thrust(j) = Tmin;
            end
        end
    
        % Combining controls
        % thrust cant be more then FT_max!
        % ft = sum(each_motor_thrust);
    
        if ft>max_Ft
            ft = max_Ft;
        elseif ft<0
            ft = 0;
        end
    end
    % if i < 100
    % for p = 1:3
    %     if torques(p,i) > Max_torque
    %         torques(p,i) = Max_torque;
    %     elseif torques(p,i) < -Max_torque
    %         torques(p,i) = -Max_torque;
    %     end
    % end
    % end
    U(:,i) = [ft; torques(:,i)];
    %% add actuator noise
    U(:,i) = add_noise(U(:,i), actuator_uncert, add_actuator_noise);

    %% Update Nonlinear Dynamics using Euler integration
    f_x = Full_f_225(X(:, i),g,Ix,Iy,Iz);
    g_x = Full_g_225(X(:, i), m,Ix,Iy,Iz);

    
    % State update using Euler integration
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (f_x + g_x * U(:, i));
    end

    instant_cost_pos(i) = pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos' * R_pos * U_pos;
    instant_cost_att(i) = Att_error(:,i)' * Q_att * Att_error(:,i) + torques(:,i)' * R_att * torques(:,i);
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + instant_cost_att(i);
   end

end
SNAC_final_time = toc;
SNAC_results.X = X;
SNAC_results.U = U;
SNAC_results.error = [pos_error;Att_error];
SNAC_results.r_initial = r_initial;
SNAC_results.inscost = [instant_cost_pos;instant_cost_att];
SNAC_results.cumcost = [cumulative_cost_pos;cumulative_cost_att];
SNAC_results.simtime = SNAC_final_time;
SNAC_error = SNAC_results.error;
SNAC_tracking_error = (SNAC_error(1,:).^2 + SNAC_error(2,:).^2 + SNAC_error(3,:).^2).^0.5;
SNAC_tracking_error_sum = dt*sum(SNAC_tracking_error,2)
SNAC_c_cost_total = sum(SNAC_results.cumcost, 1);
SNAC_c_cost_total(end-1)
if (add_sensor_noise > 0 || add_actuator_noise > 0) && add_saturation ~=1
    save("SNAC_compare_workspace_noise.mat","SNAC_results")
    disp("Saving noise only simulation")
elseif add_saturation == 1
    save("SNAC_compare_workspace_noise_saturation.mat","SNAC_results")
    disp("Saving noise and saturation simulation")
elseif add_wind == 1
    save("SNAC_compare_workspace_wind.mat","SNAC_results")
    disp("Saving wind simulation")
else 
    save("SNAC_compare_workspace.mat","SNAC_results")
    disp("Saving ideal simulation")
end

% save("SNAC_IC1.mat","SNAC_results","W_nominal", "std_div_wind", "heading_nominal", "std_div_wind_heading")

%% plot Tracking error
figure;
hold on;
plot(T(1:end-1), SNAC_tracking_error(1,1:N-1), 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Tracking Error [m]');
title('L2 Norm Tracking Error');
grid on;

% %% Plot results
% % Plot Instantaneous Cost
% figure;
% plot(T, instant_cost_pos,T,instant_cost_att, 'r', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Instantaneous Cost');
% title('Instantaneous Cost Over Time');
% legend("Position control cost","Attitude Control Cost")
% grid on;
% 
% % Plot Cumulative Cost
% figure;
% plot(T, cumulative_cost_pos,T,cumulative_cost_att, 'r', 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Cumulative Cost');
% title('Total Accumulated Cost Over Time');
% legend("Position control cost","Attitude Control Cost")
% grid on;
% 
% figure;
% titles = {'x', 'y', 'z', 'vx', 'vy', 'vz', 'phi', 'theta', 'psi', 'p', 'q', 'r'};
% for i = 1:12
%     subplot(4,3,i);
%     plot(T, X(i,:), 'b', 'LineWidth', 1.5); hold on;
%     plot(T, X_ref(i,:), 'r--', 'LineWidth', 1.5);
%     xlabel('Time (s)'); ylabel(titles{i});
%     legend('Nonlinear Model', 'Reference');
%     grid on;
% end
% sgtitle('SNAC');

figure;
plot(T, U, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Inputs');
legend('ft', '$\tau_x$', '$\tau_y$', '$\tau_z$', "Interpreter","latex");
grid on;
title('Control Inputs', "Interpreter","latex");
% 
figure;
grid on
hold on
plot3(X_ref(1,:), X_ref(2,:), -X_ref(3,:), '--', 'Linewidth', 1.5)
plot3(X(1,:), X(2,:), -X(3,:), 'Linewidth', 1.5)
title('Tracking with Rapidly Changing Payload', "Interpreter","latex")
xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
legend(["Reference trajectory", "SNAC"]);


%% rememeber to input u1 and u2 and u3 as negative into this function
function [ft, pitch, roll] = borna_sys_solve(u1, u2, u3, psi, m) % u1 = ux , u2 = uy , u3 = uz - g  
    % if psi <= 0.001 && psi >= -0.001
    %     psi = 0.001;
    % end
    a = sin(psi);
    b = cos(psi);
    A = u1 / (u3);
    B = u2 / (u3);
    C = (a*A - b*B) / ( (a^2) + (b^2));
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

function noisy_vector = add_noise(state_vector, mag, noise_percent)
    noise = (noise_percent) * mag .* randn(size(state_vector));
    noisy_vector = state_vector + noise;
end
function R = eul2rotm_zyx(phi, theta, psi)
    % Compute trigonometric values
    c_phi = cos(phi);    s_phi = sin(phi);
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi);    s_psi = sin(psi);
    
    % Construct the ZYX rotation matrix
    R = [ c_theta * c_psi,  s_phi * s_theta * c_psi - c_phi * s_psi,  c_phi * s_theta * c_psi + s_phi * s_psi;
          c_theta * s_psi,  s_phi * s_theta * s_psi + c_phi * c_psi,  c_phi * s_theta * s_psi - s_phi * c_psi;
         -s_theta,          s_phi * c_theta,                          c_phi * c_theta ];
end
function [accn_dist, alpha_dist] = wind_f_m(Wxyz, X, rho, Cd, A, r, m, Ix, Iy, Iz)

    R = eul2rotm_zyx( X(7), X(8), X(9) );
    Wxyz_body = R'*Wxyz;
    V_rel = X(4:6) - Wxyz_body;
    F_wind = -0.5 * rho * Cd .*diag(A) * (V_rel .* abs(V_rel)); % Compute wind forces
    
    F_wind_motors = repmat(F_wind, 1, 4) + 0.01*randn(3,4);

    M_wind_motors = cross(r, F_wind_motors, 1);
    M_wind = sum(M_wind_motors, 2);  
    
    accn_dist = F_wind/m;
    alpha_dist=    [M_wind(1)/Ix;
                    M_wind(2)/Iy;
                    M_wind(3)/Iz];
end
 

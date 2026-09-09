% wind modeling
% direction in the global frame: heading and incline
% magnitude is in m/s of windspeed
% drone states for the rotational matrix
%% wind testing on snac without traning: 1.1) W_nominal = 0 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std
%                                       1.2) W_nominal = 0.5 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std
%                                       1.3) W_nominal = 1 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std
%                                       1.4) W_nominal = 1.5 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std ## failure case
%% wind testing on snac with traning on wind: 2.1) W_nominal = 1 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std
%                                           2.2) W_nominal = 2 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std
%                                           2.3) W_nominal = 3 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std
%                                           2.4) W_nominal = 4 m/s w/ 2 m/s std, heading nominal = 90 degrees w/ 20 deg std
clear; clc; close all;
rng(5,"twister")
% drone parameters
dt = 0.004;
Ix = 0.3;   % Moment of inertia (kg*m^2)
Iy = 0.4;
Iz = 0.5;
m = 1;
Cd = [.8; 0.8;1.2]; % Drag coefficient
L = 0.2; % Half arm length (m)
h = 0.05;
A = [L*2*h, L*2*h, 2*L*L]; % Approximate frontal areas [x, y, z] in m^2
r = [ L,  L, -L, -L;   % X positions of motors
      L, -L, -L,  L;   % Y positions of motors
      0,  0,  0,  0 ]; % Z positions (motors at same height)
% sim time
tf = 50;
N = tf/dt;
% air properites
rho = 1.225; % Air density (kg/m^3)
% wind modeling
W = zeros(1,N);
W_nominal = 5;
W(1) = W_nominal; % nominal wind speed
heading = zeros(1,N);
incline = zeros(1,N);
incline(1) = 0;
heading_nominal = pi/2;
heading(1) = heading_nominal; % wind headng east
std_div_wind = 2; % standard divation of wind mag is 1m/s
std_div_wind_heading = deg2rad(20); % 10 degrees std of heading
std_div_wind_incline = deg2rad(0);
Tw = 20; % drift time 
T_heading = 20;
T_incline = 20;

% Test on a flight path 
load("SNAC_simulations_workspace.mat","results")
X = results.x;



for i = 1:N
    % first order wind model

    % wind mag dynamics: drift around nominal value
    W_dot = -((W(i)-W(1))/Tw) + std_div_wind*randn;
    W(i+1) = W(i) + dt*W_dot;
    % wind direction dynamics: drift around 0 degrees
    % incline_dot = -(incline(i)/T_incline) + std_div_wind_incline*randn;
    % incline(i+1) = incline(i) + dt*incline_dot;
    heading_dot = -(heading(i)/T_heading) + std_div_wind_heading*randn;
    heading(i+1) = heading(i) + dt*heading_dot;
    % NED global wind componates: find global x, y, z wind speeds 
    Wxyz(:,i) = [W(i)*cos(incline(i))*cos(heading(i));
        W(i)*cos(incline(i))*sin(heading(i));
        -W(i)*sin(incline(i))];
    
    % wind in body frame: How is the wind flowing w/r to the drone body frame velocity 
    R = eul2rotm_zyx( X(7,i), X(8,i), X(9,i));
    Wxyz_body(:,i) = inv(R)*Wxyz(:,i);
    V_rel(:,i) = X(4:6,i) - Wxyz_body(:,i);

    F_wind(:,i) = -0.5 * rho * Cd .*diag(A) * (V_rel(:,i) .* abs(V_rel(:,i))); % Compute wind forces
    
    F_wind_motors = repmat(F_wind(:,i), 1, 4) + 0.01*randn(3,4);

    M_wind_motors = cross(r, F_wind_motors, 1);
    M_wind(:,i) = sum(M_wind_motors, 2);  
    
    dist_vector(:,i) = [F_wind(:,i)/m;
                    M_wind(1,i)/Ix;
                    M_wind(2,i)/Iy;
                    M_wind(3,i)/Iz];
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
figure;
plot(X(6,:)); % Pitch over time
figure;
plot(Wxyz(3,:)); % Z component of wind
figure;
plot(Wxyz_body(3,:)); % Z component in body frame
figure; 
plot(V_rel(3,:))

% this is a generated wind vector of a 5 m/s wind with variation in incline
% and heading to apply to all the drone simulations 
save("wind_simulation.mat", "Wxyz","r","Cd","rho","A", "dist_vector","std_div_wind","std_div_wind_heading", "W_nominal","heading_nominal")
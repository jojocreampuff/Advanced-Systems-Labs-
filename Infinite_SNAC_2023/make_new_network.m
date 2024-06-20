%% train a new NN7 tat takes in Yaw data
clear; clc; close;

% DRONE MASS
m = 3.1;
% my desired inputs are [ux, uy, uz, r_psi]
% ux,uy,uz need to ranger from -100 to 100
% r_psi ranges from -pi to pi


% my desired outputs are [ft, r_psi, r_theta]


% Define the ranges for inputs
num_samples = 5;
ux_range = [-15, 15]; % Range for ux
uy_range = [-15, 15]; % Range for uy
uz_range = [-15, 15]; % Range for uz
r_psi_range = [pi, pi]; % Range for r_psi
% m_range = [0.5, 5]; % Range for mass

% Generate random samples within the specified ranges
ux = ux_range(1) + (ux_range(2) - ux_range(1)) * rand(num_samples, 1);
uy = uy_range(1) + (uy_range(2) - uy_range(1)) * rand(num_samples, 1);
uz = uz_range(1) + (uz_range(2) - uz_range(1)) * rand(num_samples, 1);
r_psi = r_psi_range(1) + (r_psi_range(2) - r_psi_range(1)) * rand(num_samples, 1);
% m = m_range(1) + (m_range(2) - m_range(1)) * rand(num_samples, 1);


% [UX, UY, UZ, R_PSI, M] = ndgrid(ux_values, uy_values, uz_values, r_psi_values, m_values);
[UX, UY, UZ, R_PSI] = ndgrid(ux, uy, uz, r_psi);
% Flatten the grids into vectors
ux = UX(:);
uy = UY(:);
uz = UZ(:);
r_psi = R_PSI(:);

% Combine the inputs into a matrix
inputs = [ux, uy, uz, r_psi]';

% Number of samples
num_samples = length(ux);

% Preallocate arrays for outputs
output = zeros(3, num_samples);

% Use parallel for loop to compute the outputs
for i = 1:num_samples
    output(:,i) = system_solver([ux(i), uy(i), uz(i)], r_psi(i), m);
end



function out = system_solver(uxyz,r_psi, m)
eqns = @(vars) [uxyz(1) - vars(1)/m.*(sin(vars(2)).*sin(r_psi) + cos(vars(2)).*cos(r_psi).*sin(vars(3)));...
                uxyz(2) - vars(1)/m.*(cos(vars(2)).*sin(r_psi).*sin(vars(3)) - cos(r_psi).*sin(vars(2)));...
                uxyz(3) - vars(1)/m.*(cos(vars(2)).*cos(vars(3)))];

% initial guess for the solution
x0 = [1; pi/4; pi/6];

% solve the system of equations
options = optimset('Display','off');

out = fsolve(eqns, x0,options);


end
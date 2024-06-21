%% test two different system solvers
clear; clc; close;
% Define the ranges for inputs
m = 5;
num_samples = 50;
ux_range = [-2, 2]; % Range for ux
uy_range = [-2, 2]; % Range for uy
uz_range = [-2, 2]; % Range for uz
r_psi_range = [-pi, pi]; % Range for r_psi
% m_range = [0.5, 5]; % Range for mass

% Generate random samples within the specified ranges
ux = ux_range(1) + (ux_range(2) - ux_range(1)) * rand(num_samples, 1);
uy = uy_range(1) + (uy_range(2) - uy_range(1)) * rand(num_samples, 1);
uz = uz_range(1) + (uz_range(2) - uz_range(1)) * rand(num_samples, 1);
r_psi = r_psi_range(1) + (r_psi_range(2) - r_psi_range(1)) * rand(num_samples, 1);


inputs = [ux, uy, uz, r_psi]';

outs_fsolve = zeros(3, num_samples);
outs_equ = zeros(3, num_samples);

parfor i = 1:num_samples

    outs_fsolve(:,i) = fsolve_system_solver([ux(i), uy(i), uz(i)], r_psi(i), m);
    outs_equ(:,i) = borna_sys_solve(ux(i), uy(i), uz(i), r_psi(i), m);


end
X = 1:1:num_samples;
% Plot results
figure;
for i = 1:3
    subplot(3, 1, i); % Create a subplot for each dimension
    plot(X, outs_equ(i,:), X, outs_fsolve(i,:) );
    xlabel('random input #');
    ylabel("solver output");
    legend("solver 1", "f solve")
end

sgtitle('Comparison of System Solvers'); % Add a super title for all subplots





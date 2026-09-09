function [xk, uk, pos_error, instant_cost_pos, cumulative_cost_pos] = alphaPerformer(iniPos, ref)

    addpath(genpath('../')); addpath(genpath('basis_funcs'))
    % load('weigths_alpha', 'Vk')
    load('workspace_alpha_2025-03-20-14-08-58-943.mat', 'Vk')
    Position_Q = diag([1000,1000,1000,1000,1000,1000]);
    Position_R = diag([1,1,1])*50; % SITL has too much control input
    constants;
    alphaDynamics;
    
    NTime = length(ref);
    NStates = 6;
    NInputs = 3;
    dt = 0.004;
    xk = zeros(NStates, NTime);
    pos_error = zeros(NStates, NTime);
    uk = zeros(NInputs, NTime);
    instant_cost_pos = zeros(1, NTime); % Instantaneous cost
    cumulative_cost_pos = zeros(1, NTime); % Integrated cost
    xk(:, 1) = iniPos;
    for k = 1 : (NTime - 1)
        pos_error(:, k) = xk(:, k) - ref(:, k);
        uk(:, k) = Vk' * lambda(pos_error(:, k));
        xk(:, k + 1) = model_sim(xk, uk, k);
        instant_cost_pos(k) = pos_error(:,k)' * Position_Q * pos_error(:,k) + uk(:, k)' * Position_R * uk(:, k);
       if k > 1
            cumulative_cost_pos(k) = cumulative_cost_pos(k-1) + instant_cost_pos(k) * dt;
       end
    end
    uk(:, end) = uk(:, end - 1);

end
function [xk, uk, att_error, instant_cost_att, cumulative_cost_att] = betaPerformer(iniAtt, ref)

    addpath(genpath('../')); addpath(genpath('basis_funcs'))
    % load('weigths_beta', 'Vk')
    load('workspace_beta_2025-03-20-14-18-08-623.mat','Vk')

    Attitude_Q = diag([10,10,10,1,1,1])*1000;
    Attitude_R = diag([1,1,1])*100;
    R_att = Attitude_R;
    Q_att = Attitude_Q;
    constants;
    betaDynamics;
    
    NTime = length(ref);
    NStates = 6;
    NInputs = 3;
    dt = 0.004;
    instant_cost_att = zeros(1, NTime); % Instantaneous cost
    cumulative_cost_att = zeros(1, NTime); % Integrated cost
    xk = zeros(NStates, NTime);
    att_error = zeros(NStates, NTime); % adding error so I can get cost
    uk = zeros(NInputs, NTime);
    
    xk(:, 1) = iniAtt;
    for k = 1 : (NTime - 1)
        att_error(:,k) = xk(:, k) - ref(:, k);
        uk(:, k) = Vk' * lambda(att_error(:,k));
        xk(:, k + 1) = model_sim(xk, uk, k); %+ randBetaCoeff*rand(6, 1);
        instant_cost_att(k) = att_error(:,k)' * Q_att * att_error(:,k) + uk(:, k)' * R_att * uk(:, k);
       if k > 1
            cumulative_cost_att(k) = cumulative_cost_att(k-1) + instant_cost_att(k) * dt;
       end
    end
    uk(:, end) = uk(:, end - 1);

end
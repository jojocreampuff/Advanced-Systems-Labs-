clc; clear; addpath(genpath('../')); addpath(genpath('basis_funcs'))

%% Parameters% Load in all the model constants
constants
dt = 0.01;
% Load in the model dynamics
betaDynamics;


% Simulation Parameters
NIters = 10000;
%% non- wind cost function
Attitude_Q = [300,300,100,10,10,10].*diag([1,1,1,1,1,1]);
Attitude_R = dt*diag([1/dt,1/dt,1/dt]);
%% wind cost function
% Attitude_Q = diag([10,10,10,1,1,1])*10000;
% Attitude_R = diag([1,1,1])*50;
% Training Parameters
Rv = Attitude_R; % Input penalizing matrix
Qv = Attitude_Q; % State penalizing matrix
Hv = Attitude_Q; % State penalizing matrix

PHI_max = pi/4; PHI_min = -pi/4;    % x1
THE_max = pi/4; THE_min = -pi/4;    % x2
PSI_max = pi/4; PSI_min = -pi/4;        % x3

p_max =  pi/4; p_min =  -pi/4;      % x4
q_max =  pi/4; q_min =  -pi/4;      % x5
r_max =  pi/4; r_min =  -pi/4;    % x6

xkMax = [PHI_max THE_max PSI_max p_max q_max r_max]';  
xkMin = [PHI_min THE_min PSI_min p_min q_min r_min]';

NPatterns = 1000;
% load("wind_simulation.mat")
% att_dist_vector = dist_vector(4:6,1:NPatterns);
itrMax = 10;            % Maximum alllowed actor iterations
convThreshold = 0.001;  % Actor's convergence threshold

%% Model


%% Formulation

% N Vars 
NStates = 6;
NInputs = 3;
NActorNeurons = length(lambda(ones(NStates, 1)));
NCriticNeurons = length(kappa(ones(NStates, 1)));

skGen = @() (2*(rand(NStates, NPatterns) > 0.5) - 1) .* (xkMin + (xkMax - xkMin).*rand(NStates, NPatterns));  % Xpatterns

% Penalizing Matrices
Rbar = timeStep*Rv;
Qbar = timeStep*Qv;
Hbar = timeStep*Hv;

% Variable Initialization
sk = skGen();
Kappa = kappa(sk);

% Jk = 0.5*sum(diag(Hbar).*sk.^2);

Vk =  rand(NActorNeurons, NInputs);  % Actor Weights
% Wk = (Kappa*Kappa')\Kappa*Jk';       % Critic Weights   [[1]]
Wk = 0*randn(NActorNeurons, 1);
Error = zeros(1, NIters);
% Error(1) = mae(Wk.' * Kappa - Jk);



tic;
for k = 1:NIters   % [[2]]


    % Inner Loop
    for i = 1:itrMax    % [[3]]
        sk = skGen();
        Lambda = lambda(sk);

        uk = Vk'*Lambda;
        sn = f_hat(sk) + g_hat_1(sk).*uk(1, :) + g_hat_2(sk).*uk(2, :) + g_hat_3(sk).*uk(3, :);
        %% adding wind disturbence
        % sn(4:6,:) = sn(4:6,:) + att_dist_vector;
        del = [(grad_kappa_1(sn)*Wk)'; (grad_kappa_2(sn)*Wk)'; (grad_kappa_3(sn)*Wk)'; (grad_kappa_4(sn)*Wk)'; (grad_kappa_5(sn)*Wk)'; (grad_kappa_6(sn)*Wk)'];

        uk_ip = -Rbar^-1*[sum(g_hat_1(sk).*del); sum(g_hat_2(sk).*del); sum(g_hat_3(sk).*del)];
        Vk_ip = (Lambda*Lambda')\Lambda*uk_ip';

        actorDelta = norm(Vk_ip - Vk, 2);
        Vk = Vk_ip;

        if  actorDelta <= convThreshold  % [[5]]
            break
        end
    end

    sk = skGen();
    Lambda = lambda(sk);
    Kappa = kappa(sk);

    % VEC
    uk = Vk'*Lambda;
    sn = f_hat(sk) + g_hat_1(sk).*uk(1, :) + g_hat_2(sk).*uk(2, :) + g_hat_3(sk).*uk(3, :);
    %% adding wind disturbence
    % sn(4:6,:) = sn(4:6,:) + att_dist_vector;
    Jk = 0.5*sum(diag(Qbar).*sk.^2) + 0.5*sum(diag(Rbar).*uk.^2) + Wk'*kappa(sn);  % [[7]]
    Wk = (Kappa*Kappa')\Kappa*Jk';

    Error(k) = mae(Wk.' * Kappa - Jk);

    if isnan(Vk)
        fprintf('Diverged\n')
        break
    end

    VV(k, :, :) = Vk;
    WW(k, :) = Wk;

end
toc;

windowSize = 10; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;

error_filtered = filter(b, a, Error);

figure; hold on; grid on;
set(gca,'linewidth',1);
set(gca,'GridLineStyle',':')
set(gca,'GridAlpha',0.7)
set(gca,'GridColor',[0 0 0])
plot(error_filtered, 'b-')
xlabel("No. Iterations"), ylabel("MAE of Training");
% ylim([0 .4])

figure; 
set(gca,'linewidth',1);
set(gca,'GridLineStyle',':')
set(gca,'GridAlpha',0.7)
set(gca,'GridColor',[0 0 0])
subplot(2,1,1)
hold on; grid on;
for i = 1:NInputs
    for j = 1:NActorNeurons
        plot(VV(:, j, 1), '-')
    end
end
xlabel('No. Iterations', 'Interpreter', 'latex');
ylabel("$\widehat{W}_c$", 'Interpreter', 'latex');
subplot(2,1,2)
hold on; grid on;
for i = 1:NActorNeurons
    plot(WW(:, i))
end
xlabel('No. Iterations', 'Interpreter', 'latex');
ylabel("$\widehat{W}_a$", 'Interpreter', 'latex');


% save(['workspace_beta_' datestr(datetime, 'yyyy-mm-dd-HH-MM-SS-FFF') '.mat'], 'Vk')
% save("ac_train.mat")
save("z_offline_AC_att_NEW.mat")
% load("ac_train.mat")
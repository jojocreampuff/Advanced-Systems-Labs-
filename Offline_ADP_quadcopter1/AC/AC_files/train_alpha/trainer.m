clc; clear; addpath(genpath('../')); addpath(genpath('basis_funcs'))

%% Parameters
% load("wind_simulation.mat", "dist_vector")

% Simulation Parameters
NIters = 5000;
timeStep = 0.01; % Simulation timestep
dt = timeStep;
Position_Q = 10*diag([1,1,1,1,1,1])/dt;
Position_R = 10*diag([1,1,1])/dt; 

xkMax = [10 10 10 10 10 10]';  
xkMin = [-10 -10 -10 -10 -10 -10]';

NPatterns = 1000;
% pos_dist_vector = dist_vector(1:3,1:NPatterns);
itrMax = 10;            % Maximum alllowed actor iterations
convThreshold = 0.0001;  % Actor's convergence thresholdIll send 

% Load in all the model constants
% Shorthand trig functions
s = @sin; c = @cos; t = @tan;

grav = 9.81;    % Gravity (m/s^2)
mass = 1;       % Mass (kg)

% b = 5;   % Thrust Factor (kg*m)
% l = 1;   % Distance between rotors (m)
% d = 0.5; % Drag Factor (kg*m)

% Moments of Inertia (kg*m^2)
Ix = 0.3;   % Moment of inertia (kg*m^2)
Iy = 0.4;
Iz = 0.5;
m = 1;

% Load in the model dynamics
alphaDynamics

%% Formulation

% N Vars 
NStates = 6;
NInputs = 3;
NActorNeurons = length(lambda(ones(NStates, 1)));
NCriticNeurons = length(kappa(ones(NStates, 1)));

skGen = @() (2*(rand(NStates, NPatterns) > 0.5) - 1) .* (xkMin + (xkMax - xkMin).*rand(NStates, NPatterns));  % Xpatterns

% Penalizing Matrices
Rbar = timeStep*Position_R;
Qbar = timeStep*Position_Q;
Hbar = timeStep*Position_Q;

% Variable Initialization
sk = skGen();
Kappa = kappa(sk);

Jk = 0.5*sum(diag(Hbar).*sk.^2);

Vk =  rand(NActorNeurons, NInputs);  % Actor Weights
% Wk = (Kappa*Kappa')\Kappa*Jk';       % Critic Weights   [[1]]
Wk = rand(NActorNeurons, 1);

Error = zeros(1, NIters);

tic;
for k = 1:NIters   % [[2]]

    % Inner Loop
    for i = 1:itrMax    % [[3]]
        sk = skGen();
        Lambda = lambda(sk);

        uk = Vk'*Lambda;
        sn = f_hat(sk) + g_hat_1(sk).*uk(1, :) + g_hat_2(sk).*uk(2, :) + g_hat_3(sk).*uk(3, :);
        %% adding wind disturbence
        % sn(4:6,:) = sn(4:6,:) + pos_dist_vector;
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
    %% Adding wind disturbance
    % sn(4:6,:) = sn(4:6,:) + pos_dist_vector;

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
% ylim([0 1e-11])

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


% figure; 
% set(gca,'linewidth',1);
% set(gca,'GridLineStyle',':')
% set(gca,'GridAlpha',0.7)
% set(gca,'GridColor',[0 0 0])
% hold on; grid on;
% for i = 1:NInputs
%     for j = 1:NActorNeurons
%         plot(VV(:, j, 1), '-')
%     end
% end
% xlabel('No. Iterations', 'Interpreter', 'latex');
% ylabel("$\widehat{W}_a$", 'Interpreter', 'latex');
% 
% 
% figure; hold on; grid on;
% set(gca,'linewidth',1);
% set(gca,'GridLineStyle',':')
% set(gca,'GridAlpha',0.7)
% set(gca,'GridColor',[0 0 0])
% for i = 1:NActorNeurons
%     plot(WW(:, i))
% end

% save(['workspace_alpha_' datestr(datetime, 'yyyy-mm-dd-HH-MM-SS-FFF') '.mat'], 'Vk')
save("z_offline_AC_pos_MORE_R.mat")
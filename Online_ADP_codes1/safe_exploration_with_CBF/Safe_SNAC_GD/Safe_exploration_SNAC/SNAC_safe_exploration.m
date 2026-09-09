%% SNAC Tracking with safe exploration with CBF
%% now testing with small unsafe region
clear; clc; close all;
global Q R alpha N_states N_neurons tf amp_mult N_R theta_star u_max
%% =======================
%  Params / Setup
% =======================
params = struct();
% --- training ---
params.tf   = 900;
params.alpha      = 15;      % learning gain
params.amp_mult   = 1.0;         % exploration amplitude multiplier
params.train_frac = 0.90;      % exploration active for first 90% of tf
params.cb = 0.0009;             % safe exploration control 
% --- costs ---
params.R = .1;
params.Q = diag([100, 5]);
% --- barrier params (soft penalty only) ---
params.barrier.gamma = 1e-2;
% params.barrier.c     = [0; 0];
% params.barrier.r     = 1.5;
params.barrier.c     = [-5; -5];
params.barrier.r     = 6.1;
params.barrier.clip = 0;
delta_h = 0.3;  % tune (units of h)
% ----- ODE solver tol -------
ode_opts = odeset('RelTol',1e-6,'AbsTol',1e-7,'MaxStep',0.1);

% --- dimensions ---
params.nx = 2;
params.nR = params.nx;
params.nz = params.nx + params.nR;   % z = [e; r] => nz = 4
params.nPhi = length(PHI(ones(params.nx,1), ones(params.nx,1)));

% --- normalization / saturation knobs ---
params.u_max     = 20;           % saturate control input
params.THETA_MAX = 1e4;          % freeze if ||theta|| too large
params.THETA_DOT_MAX = 1e4;      % clamp theta_dot
params.EPS_REG   = 1e-6;         % regularize norm factor in sigma normalization

% --- linear model used by CARE target ---
A = [0  1;
    -1  1];
B = [0; 1];
[P, ~, ~] = icare(A, B, params.Q, params.R);

% --- Build theta_star (consistent mapping for lambda_e = 2*P*e using basis e1,e2) ---
W_star = zeros(params.nPhi, params.nz);
W_star(1,1:2) = [P(1,1), P(1,2)];
W_star(2,1:2) = [P(2,1), P(2,2)];
params.theta_star = reshape(W_star, [], 1);

% --- initial weights ---
W_init = zeros(params.nPhi, params.nz);
W_init(1,1:2) = [1, 1];
W_init(2,1:2) = [1, 1];
theta_init = reshape(W_init, [], 1);
% --- closed-loop simulation ---
params.T_sim = 10;
params.dt    = 0.01;

%% (1) train unsafe network (for comparision)
fprintf("Training UNSAFE (barrier OFF)...\n");
Q = params.Q;   R = params.R;   alpha = params.alpha;   N_states = params.nx;
N_neurons = params.nPhi;     tf = params.tf;    amp_mult = params.amp_mult; 
N_R = params.nR; theta_star = params.theta_star; u_max = params.u_max;
X0 = zeros(params.nx,1);
initials = [X0;theta_init];
tic
[t1,x1]= ode23(@online_snac_training_tracking, [0 params.tf], initials); 
% extract states
state_history1 = x1(:, 1:params.nx);
error_history1 = 0*state_history1;
for j = 1:length(t1)
    [ref_training, ~] = ref_refdot(t1(j));
    error_history1(j,:) = state_history1(j,:) - ref_training';
end
W_history1 = x1(:,params.nx+1:end);
W_unsafe = W_history1(end,:);
W_unsafe = reshape(W_unsafe, params.nPhi,params.nx+params.nR);
unsafe_train_time = toc
%% (2) Train kinda safe network (with b(x) inside cost w/o ub)
% fprintf("Training kinda SAFE (barrier ON NO ub)...\n");
% tic
% train_kinda_Safe = run_training(theta_init, params,  false, ode_opts); % THIS TRAINING IS NOT POSSIBLE CANNOT ADD IT TO THE PAPER
% kinda_safe_train_time = toc
%% (3) Train safe network (with b(x) inside cost and ub)
fprintf("Training SAFE (barrier ON)...\n");
tic
trainSafe = run_training(theta_init, params, true, ode_opts);
safe_train_time = toc

%%  Plot training (use the step size to reduce figure filesize)
plot_training(t1',error_history1',W_history1',"Training Without Safety",10)
% plot_training(train_kinda_Safe.t',train_kinda_Safe.err',train_kinda_Safe.theta_hist',"Training With Safety", 200)
plot_training(trainSafe.t',trainSafe.err',trainSafe.theta_hist',"Training With Safety", 200)

%%  Run time: Closed-loop sim (ode45)
    % case 1: Tracking problem with NO safety
    % case 2: Tracking Problem with safe exploration with ub and B(x) in cost function 
    % case 3: Tracking problem with ub added only at runtime (shielding)
Case1_sim   = simulate_closed_loop(W_unsafe, params, 0);
Case2_sim   = simulate_closed_loop(trainSafe.W_final,   params, 1);
Case3_sim   = simulate_closed_loop(W_unsafe,   params, 1);

fprintf("\n=== Closed-loop Tracking RMSE ===\n");
fprintf("  Case 1: Unsafe policy: %.6f\n", Case1_sim.RMSE);
fprintf("  Case 2: Safe   policy w/ safe exploration: %.6f\n", Case2_sim.RMSE);
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %.6f\n", Case3_sim.RMSE);

fprintf("\n=== Closed-loop Tracking Control Cost ===\n");
fprintf("  Case 1: Unsafe policy: %.6f\n", Case1_sim.control_cost);
fprintf("  Case 2: Safe   policy w/ safe exploration: %.6f\n", Case2_sim.control_cost);
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %.6f\n", Case3_sim.control_cost);

fprintf("\n=== Closed-loop Tracking Cost ===\n");
fprintf("  Case 1: Unsafe policy: %.6f\n", Case1_sim.tracking_cost);
fprintf("  Case 2: Safe   policy w/ safe exploration: %.6f\n", Case2_sim.tracking_cost);
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %.6f\n", Case3_sim.tracking_cost);

fprintf("\n=== Safety Violations (h < 0 in X timesteps) ===\n");
fprintf("  Case 1: Unsafe policy: %d / %d\n", Case1_sim.safety_viol, length(Case1_sim.h_log));
fprintf("  Case 2: Safe   policy: %d / %d\n", Case2_sim.safety_viol,   length(Case2_sim.h_log));
fprintf("  Case 3: Unsafe policy w/ ub as shielding: %d / %d\n", Case3_sim.safety_viol,   length(Case3_sim.h_log));


%% =======================
%  Plots
% =======================
figure; hold on; grid on;
plot(Case1_sim.time, Case1_sim.h_log, 'Color', [0 0 0],"LineStyle","--", 'Marker', 'x', 'LineWidth', 1, ...
    'MarkerIndices', 1:50:length(Case1_sim.time), 'MarkerSize', 5);
plot(Case2_sim.time,   Case2_sim.h_log, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(Case2_sim.time), 'MarkerSize', 5);
plot(Case3_sim.time, Case3_sim.h_log, 'Color', [1 0 0],"LineStyle","--", 'Marker', 'v', 'LineWidth', 1, ...
    'MarkerIndices', 1:100:length(Case3_sim.time), 'MarkerSize', 5); 
xlabel("Time (s)"); ylabel("error");
yline(0,'g-','LineWidth',1.0);
xlabel("Time (s)", 'Interpreter','latex');
ylabel("$h(x)$",   'Interpreter','latex');
legend("Unsafe","Safe Exploration","Unsafe w/ ub shielding",'Location','best');
title("Barrier margin $h(x)$", 'Interpreter','latex');

theta_circ = linspace(0, 2*pi, 200);
bx = params.barrier.c(1) + params.barrier.r*cos(theta_circ);
by = params.barrier.c(2) + params.barrier.r*sin(theta_circ);

% figure; hold on; grid on;
% plot(Case1_sim.x(1,:), Case1_sim.x(2,:), 'Color', [0 0 0],"LineStyle","--", 'Marker', 'x', 'LineWidth', 1, ...
%     'MarkerIndices', 1:700:length(Case1_sim.time), 'MarkerSize', 5);
% plot(Case2_sim.x(1,:),   Case2_sim.x(2,:), 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
%      'MarkerIndices', 1:500:length(Case2_sim.time), 'MarkerSize', 5);
% plot(Case3_sim.x(1,:), Case3_sim.x(2,:), 'Color', [1 0 0],"LineStyle","--", 'Marker', 'v', 'LineWidth', 1, ...
%     'MarkerIndices', 1:400:length(Case3_sim.time), 'MarkerSize', 5);
% plot(Case1_sim.ref(1,:), Case1_sim.ref(2,:),"Color","g","LineStyle","-","LineWidth",1);
% plot(bx, by, "Color","r", "LineStyle","-",'LineWidth',1);
% xlabel("x_1"); ylabel("x_2");
% xlim([-1.5 1.5])
% ylim([-2 2])
% legend("Unsafe","Safe Exploration","Unsafe w/ ub shielding","Reference","Safety boundary",'Location','best');
% title("State-space tracking + safety boundary");

figure; hold on; grid on;

% Unsafe (baseline)
plot(Case1_sim.x(1,:), Case1_sim.x(2,:), ...
    'k--', 'LineWidth', 1.5);

% Safe exploration (proposed)
plot(Case2_sim.x(1,:), Case2_sim.x(2,:), ...
    'b-', 'LineWidth', 1.8);

% Unsafe + shielding
plot(Case3_sim.x(1,:), Case3_sim.x(2,:), ...
    'Color', [0.4 0.4 0.4], 'LineStyle','-.', 'LineWidth', 1.5);

% Reference
plot(Case1_sim.ref(1,:), Case1_sim.ref(2,:), ...
    'k:', 'LineWidth', 1.5);

% Safety boundary
plot(bx, by, ...
    'r-', 'LineWidth', 2);

xlabel('$x_1$', 'Interpreter','latex');
ylabel('$x_2$', 'Interpreter','latex');

xlim([-1.5 1.5])
ylim([-2 2])

legend({'Unsafe policy', ...
        'Safe exploration (proposed)', ...
        'Shielded unsafe policy', ...
        'Reference trajectory', ...
        'Safety boundary'}, ...
        'Location','best', ...
        'Interpreter','latex');

set(gca,'FontSize',12,'LineWidth',1);
box on;

figure; hold on; grid on;
plot(Case1_sim.time, Case1_sim.L2_err, 'Color', [0 0 0],"LineStyle","--", 'Marker', 'x', 'LineWidth', 1, ...
    'MarkerIndices', 1:50:length(Case1_sim.time), 'MarkerSize', 5);
plot(Case2_sim.time,   Case2_sim.L2_err, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(Case2_sim.time), 'MarkerSize', 5);
plot(Case3_sim.time, Case3_sim.L2_err, 'Color', [1 0 0],"LineStyle","--", 'Marker', 'v', 'LineWidth', 1, ...
    'MarkerIndices', 1:100:length(Case3_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("error");
ylim([0 .2])
legend("Unsafe","Safe Exploration","Unsafe w/ ub shielding",'Location','best');
title("Error plot");

figure; hold on; grid on;
plot(Case1_sim.time, Case1_sim.u, 'Color', [0 0 0],"LineStyle","--", 'Marker', 'x', 'LineWidth', 1, ...
    'MarkerIndices', 1:50:length(Case1_sim.time), 'MarkerSize', 5);
plot(Case2_sim.time,   Case2_sim.u, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(Case2_sim.time), 'MarkerSize', 5);
plot(Case3_sim.time, Case3_sim.u, 'Color', [1 0 0],"LineStyle","--", 'Marker', 'v', 'LineWidth', 1, ...
    'MarkerIndices', 1:100:length(Case3_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("Control");
legend("Unsafe","Safe Exploration","Unsafe w/ ub shielding",'Location','best');
title("Control Plot");

figure;
subplot(3,1,1)
plot(Case1_sim.time, Case1_sim.u, 'Color', [0 0 0],"LineStyle","--", 'Marker', 'x', 'LineWidth', 1, ...
    'MarkerIndices', 1:50:length(Case1_sim.time), 'MarkerSize', 5); hold on; grid on; xlabel("Time (s)"); ylabel("Control");
legend("Unsafe",'Location','best');
subplot(3,1,2)
plot(Case2_sim.time,   Case2_sim.u, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(Case2_sim.time), 'MarkerSize', 5); hold on; grid on; xlabel("Time (s)"); ylabel("Control");
legend("Safe Exploration",'Location','best');
subplot(3,1,3)
plot(Case3_sim.time, Case3_sim.u, 'Color', [1 0 0],"LineStyle","--", 'Marker', 'v', 'LineWidth', 1, ...
    'MarkerIndices', 1:100:length(Case3_sim.time), 'MarkerSize', 5); hold on; grid on; xlabel("Time (s)"); ylabel("Control");
legend("Unsafe w/ ub Shielding",'Location','best');
title("Control Plot");

% saveFigures("SNAC_safe_model_based")
% save("SNAC_safe_model_based_barrier_2")
%% =======================
%  Local functions
%% =======================
function out = run_training(theta_init, params, useSafetyExploration, ode_opts)
    x0 = zeros(params.nx,1);
    y0 = [x0; theta_init];
    
    if useSafetyExploration == 1 
        odefun = @(t,y) online_snac_ode_safe_exploration(t, y, params);
    else
        odefun = @(t,y) online_snac_ode_kinda_safe(t, y, params);
    end


    [t, y] = ode23(odefun, [0 params.tf], y0, ode_opts);

    x_hist     = y(:, 1:params.nx);
    theta_hist = y(:, params.nx+1:end);

    r_hist = zeros(params.nx, length(t));
    for k = 1:length(t)
        [r_hist(:,k), ~] = ref_refdot(t(k));
    end
    err = x_hist - r_hist';

    theta_final = theta_hist(end,:).';
    W_final = reshape(theta_final, params.nPhi, params.nz);

    out = struct();
    out.t = t;
    out.x_hist = x_hist;
    out.r_hist = r_hist;
    out.err = err;
    out.theta_hist = theta_hist;
    out.W_final = W_final;
end

function dy = online_snac_ode_safe_exploration(t, y, params)
    x     = y(1:params.nx);
    theta = y(params.nx+1:end);

    if any(~isfinite(x)) || any(~isfinite(theta))
        error("Non-finite state/theta at t=%.6f.", t);
    end

    W = reshape(theta, params.nPhi, params.nz);

    % reference, error, features
    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];

    % plant (Van der Pol)
    f_vec = [x(2); (1-x(1)^2)*x(2) - x(1)];
    g     = [0; 1];

    % augmented dynamics z=[e;r]
    Fz = [f_vec - r_dot;
          r_dot];
    Gz = [g;
          zeros(size(g))];   % 4x1

    % nominal control from SNAC policy
    u_nom = -0.5*(params.R^-1) * (Gz') * (W') * phi;
    [Bbar, gradB, h] = B_x(x,params);

    if h <= 0 
        persistent viol_count;
        if isempty(viol_count), viol_count = 0; end
        viol_count = viol_count + 1;
        if mod(viol_count, 100) == 1
            fprintf("Safety Violation #%f: h_raw=%.4f, x=[%.3f %.3f]\n", ...
                    viol_count, h, x(1), x(2));
        end
    end
    
    u_bar  = -params.cb * (params.R^-1) * (g') * gradB;        % scalar
    u_nom  = u_nom + u_bar;
    u_nom  = max(min(u_nom, params.u_max), -params.u_max);

    % ---- Option A: SAFE exploration by gating u_exc near boundary ----
    u_exc = u_excite(t, params.amp_mult);
    delta_h = 0.3;  % tune (units of h)
    gate = min(max(h / delta_h, 0), 1);

    if t <= params.tf*params.train_frac
        u = u_nom + gate*u_exc;
    else
        u = u_nom;
    end
    u = max(min(u, params.u_max), -params.u_max);

    z_dot = Fz + Gz*u;

    Qz = blkdiag(params.Q, zeros(params.nx));

    r_x_u = Z.'*Qz*Z + u.'*params.R*u + Bbar;

    sigma = kron(z_dot, phi);

    % normalized factor (regularized)
    norm_sq  = 1 + sigma.'*sigma + params.EPS_REG;
    norm_fac = 1 / (norm_sq^2);

    % -------- theta_dot --------
    if t <= params.tf*params.train_frac
        if norm(theta) > params.THETA_MAX
            % warning("||theta||=%.2e > %.0e at t=%.4f — freezing theta_dot this step.", ...
            %         norm(theta), params.THETA_MAX, t);
            theta_dot = zeros(size(theta));
        else
            bellman_err = theta.'*sigma + r_x_u;


            Nmat  = kron(eye(params.nz), phi');       % (nz x nz*nPhi)
            Mmat  = Gz*(params.R^-1)*Gz';            % (nz x nz)
            inner = Nmat' * Mmat * Nmat;             % (nz*nPhi x nz*nPhi)

            theta_star  = params.theta_star;
            scalar_star = (theta_star.' * inner * theta_star);
            scalar_th   = (theta.'      * inner * theta);

            F1 = 0.5 * inner * theta_star ...
               - 0.5 * sigma * norm_fac * scalar_star ...
               + 0.25 * sigma * norm_fac * scalar_star ...
               - norm_fac * (sigma*sigma') * theta_star;

            F2 = norm_fac * (sigma*sigma');

            A_extra = (params.alpha^-1) * ( F1 + F2*theta + 0.25*sigma*norm_fac*scalar_th );

            theta_dot = -params.alpha * norm_fac * sigma * bellman_err +  A_extra;
        end
    else
        theta_dot = zeros(size(theta));
    end

    % clamp theta_dot
    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        % warning("theta_dot clamped at t=%.4f (max |theta_dot|=%.2e).", ...
        %         t, max(abs(theta_dot)));
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    % plant derivative
    x_dot = f_vec + g*u;

    dy = [x_dot; theta_dot];
end


function dy = online_snac_ode_kinda_safe(t, y, params)
    x     = y(1:params.nx);
    theta = y(params.nx+1:end);

    if any(~isfinite(x)) || any(~isfinite(theta))
        error("Non-finite state/theta at t=%.6f.", t);
    end

    W = reshape(theta, params.nPhi, params.nz);

    % reference, error, features
    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];

    % plant (Van der Pol)
    f_vec = [x(2); (1-x(1)^2)*x(2) - x(1)];
    g     = [0; 1];

    % augmented dynamics z=[e;r]
    Fz = [f_vec - r_dot;
          r_dot];
    Gz = [g;
          zeros(size(g))];   % 4x1

    % nominal control from SNAC policy
    u = -0.5*(params.R^-1) * (Gz') * (W') * phi;
    [Bbar, gradB, h] = B_x(x,params);

    if h <= 0 
        persistent viol_count;
        if isempty(viol_count), viol_count = 0; end
        viol_count = viol_count + 1;
        [Bbar, gradB, h] = B_x_clipped(x,params);
        if mod(viol_count, 500) == 1
            fprintf("Safety Violation #%f: h_raw=%.4f, x=[%.3f %.3f]\n", ...
                    viol_count, h, x(1), x(2));
        end
    end
    
    u  = max(min(u, params.u_max), -params.u_max);

    % ---- Option A: SAFE exploration by gating u_exc near boundary ----
    u_exc = u_excite(t, params.amp_mult);
    delta_h = 0.3;  % tune (units of h)
    gate = min(max(h / delta_h, 0), 1);

    if t <= params.tf*params.train_frac
        u = u + gate*u_exc;
    end
    u = max(min(u, params.u_max), -params.u_max);

    z_dot = Fz + Gz*u;

    Qz = blkdiag(params.Q, zeros(params.nx));

    r_x_u = Z.'*Qz*Z + u.'*params.R*u + Bbar;

    sigma = kron(z_dot, phi);

    % normalized factor (regularized)
    norm_sq  = 1 + sigma.'*sigma + params.EPS_REG;
    norm_fac = 1 / (norm_sq^2);

    % -------- theta_dot --------
    if t <= params.tf*params.train_frac
        if norm(theta) > params.THETA_MAX
            % warning("||theta||=%.2e > %.0e at t=%.4f — freezing theta_dot this step.", ...
            %         norm(theta), params.THETA_MAX, t);
            theta_dot = zeros(size(theta));
        else
            bellman_err = theta.'*sigma + r_x_u;


            Nmat  = kron(eye(params.nz), phi');       % (nz x nz*nPhi)
            Mmat  = Gz*(params.R^-1)*Gz';            % (nz x nz)
            inner = Nmat' * Mmat * Nmat;             % (nz*nPhi x nz*nPhi)

            theta_star  = params.theta_star;
            scalar_star = (theta_star.' * inner * theta_star);
            scalar_th   = (theta.'      * inner * theta);

            F1 = 0.5 * inner * theta_star ...
               - 0.5 * sigma * norm_fac * scalar_star ...
               + 0.25 * sigma * norm_fac * scalar_star ...
               - norm_fac * (sigma*sigma') * theta_star;

            F2 = norm_fac * (sigma*sigma');

            A_extra = (params.alpha^-1) * ( F1 + F2*theta + 0.25*sigma*norm_fac*scalar_th );

            theta_dot = -params.alpha * norm_fac * sigma * bellman_err +  A_extra;
        end
    else
        theta_dot = zeros(size(theta));
    end

    % clamp theta_dot
    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        % warning("theta_dot clamped at t=%.4f (max |theta_dot|=%.2e).", ...
        %         t, max(abs(theta_dot)));
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    % plant derivative
    x_dot = f_vec + g*u;

    dy = [x_dot; theta_dot];
end


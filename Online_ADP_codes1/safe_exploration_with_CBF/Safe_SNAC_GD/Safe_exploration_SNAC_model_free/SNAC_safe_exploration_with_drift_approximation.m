%% SNAC Tracking + SAFE exploration + Drift Learning (Linear-in-Parameters NN) + SigmaHat
clear; clc; close all;
params = struct();
% --- training ---
params.tf_train   = 900;
params.alpha      = 10;       % SNAC learning gain
params.amp_mult   = 1;        % exploration amplitude multiplier
params.train_frac = 0.90;     % exploration active for first 90% of tf_train
params.cb         = 0.0009;   % safe exploration control
% --- costs ---
params.R = 0.1;
params.Q = diag([500, 1]);
% --- barrier params (soft penalty only) ---
params.barrier.gamma = 1e-2;
params.barrier.c     = [0; 0];
params.barrier.r     = 1.5;
delta_h = 0.3;

ode_opts = odeset('RelTol',1e-6,'AbsTol',1e-7,'MaxStep',0.1);

% --- dimensions ---
params.nx = 2;
params.nR = params.nx;
params.nz = params.nx + params.nR;   % z = [e; r] => nz = 4
params.nPhi = length(PHI(ones(params.nx,1), ones(params.nx,1)));

%% =======================
%  Drift NN params: fhat(x)=Hhat' * psi(.)
% =======================
params.nn.use_e_input = false;   % false: psi(x), true: psi(e)
params.nn.gammaH      = .01;       % learning gain for Hhat
params.nn.H_MAX       = 1e4;     % freeze if ||Hhat|| too large
params.nn.H_DOT_MAX   = 1e4;     % clamp Hhat_dot
params.nn.lambda      = 0;
params.nPsi = length(PSI_drift_approx(ones(params.nx,1)));

% --- normalization / saturation knobs ---
params.u_max     = 20;           % saturate control input
params.THETA_MAX = 1e4;          % freeze if ||theta|| too large
params.THETA_DOT_MAX = 1e4;      % clamp theta_dot
params.EPS_REG   = 1e-6;         % regularize norm factor in sigma normalization

% --- closed-loop simulation ---
params.T_sim = 10;
params.dt    = 0.01;

%% theta star guess 
A = [0  1; -1  1];
B = [0; 1];
[P, ~, ~] = icare(A, B, params.Q, params.R);
W_star = zeros(params.nPhi, params.nz);
W_star(1,1:2) = [P(1,1), P(1,2)];
W_star(2,1:2) = [P(2,1), P(2,2)];
params.theta_star = reshape(W_star, [], 1);

%% =======================
%  Initial conditions
%% =======================
W_init = zeros(params.nPhi, params.nz);
W_init(1,1:2) = [1, 1];
W_init(2,1:2) = [1, 1];
theta_init = reshape(W_init, [], 1);

xhat0     = zeros(params.nx,1);
Hhat0     = zeros(params.nPsi, params.nx); %
Hhat0(1:3,:) = [ 0  -1 ; 1   1; 0  -1]; % H^* optimal weights 
Hhat0 = .3*randn(params.nPsi, params.nx) .* Hhat0;
Hhat_init = reshape(Hhat0, [], 1);


%% =======================
%  Train SAFE (barrier ON)
% =======================
fprintf("Training SAFE (barrier ON)\n");
tic
trainSafe = run_training(theta_init, xhat0, Hhat_init, params, true, ode_opts);
safe_train_time = toc

%% =======================
%  Train UNSAFE (barrier OFF)
% =======================
fprintf("Training UNSAFE (barrier OFF)\n");
tic
trainUnsafe = run_training(theta_init, xhat0, Hhat_init, params, false, ode_opts);
unsafe_train_time = toc

%% =======================
%  Plot training (use the step size to reduce figure filesize)
plot_training(trainUnsafe.t',trainUnsafe.err',trainUnsafe.theta_hist',"Training Without Safety",100)
plot_training(trainSafe.t',trainSafe.err',trainSafe.theta_hist',"Training With Safety", 200)

plot_drift_approx(trainSafe,   "Drift Approx During SAFE Exploration Training", 100);
plot_drift_approx(trainUnsafe,   "Drift Approx During UNSAFE Exploration Training", 200);

%% =======================
%  Closed-loop sim (ode45) using learned W only
Case1_sim   = simulate_closed_loop(trainUnsafe.W_final, params, 0);
Case2_sim   = simulate_closed_loop(trainSafe.W_final,   params, 1);
Case3_sim   = simulate_closed_loop(trainUnsafe.W_final,  params, 1);

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

figure; hold on; grid on;
plot(Case1_sim.x(1,:), Case1_sim.x(2,:), 'Color', [0 0 0],"LineStyle","--", 'Marker', 'x', 'LineWidth', 1, ...
    'MarkerIndices', 1:700:length(Case1_sim.time), 'MarkerSize', 5);
plot(Case2_sim.x(1,:),   Case2_sim.x(2,:), 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:500:length(Case2_sim.time), 'MarkerSize', 5);
plot(Case3_sim.x(1,:), Case3_sim.x(2,:), 'Color', [1 0 0],"LineStyle","--", 'Marker', 'v', 'LineWidth', 1, ...
    'MarkerIndices', 1:400:length(Case3_sim.time), 'MarkerSize', 5);
plot(Case2_sim.ref(1,:), Case2_sim.ref(2,:),"Color","g","LineStyle","-","LineWidth",1);
plot(bx, by, "Color","r", "LineStyle","-",'LineWidth',1);
xlabel("x_1"); ylabel("x_2");
legend("Unsafe","Safe Exploration","Unsafe w/ ub shielding","Reference","Safety boundary",'Location','best');
title("State-space tracking + safety boundary");

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

%% =======================
%  Local functions
%% =======================

function out = run_training(theta_init, xhat0, Hhat_init, params, useSafety, ode_opts)
    x0 = zeros(params.nx,1);

    % y = [x; theta; xhat; Hhat]
    y0 = [x0; theta_init; xhat0; Hhat_init];

    if useSafety == 1
        odefun = @(t,y) online_snac_ode(t, y, params);
    else
        odefun = @(t,y) online_snac_ode_UNSAFE(t, y, params);
    end

    [t, y] = ode23(odefun, [0 params.tf_train], y0, ode_opts);

    nx   = params.nx;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;

    % parse y columns for logging
    % [ x(2), theta(nPhi*nz), xhat(2), Hhat(nPsi*nx) ]
    x_hist     = y(:, 1:nx);
    theta_hist = y(:, nx+1 : nx + nPhi*nz);
    xhat_hist  = y(:, nx + nPhi*nz + 1 : nx + nPhi*nz + nx);
    Hhat_hist  = y(:, nx + nPhi*nz + nx + 1 : end);
 H_final = reshape(Hhat_hist(end,:).', params.nPsi, params.nx);

    r_hist = zeros(params.nx, length(t));
    for k = 1:length(t)
        [r_hist(:,k), ~] = ref_refdot(t(k));
    end
    err = x_hist - r_hist';

    E_hist = x_hist - xhat_hist;
    E_norm = vecnorm(E_hist, 2, 2);

    theta_final = theta_hist(end,:).';
    W_final = reshape(theta_final, params.nPhi, params.nz);

    out = struct();
    out.t = t;
    out.x_hist = x_hist;
    out.r_hist = r_hist;
    out.err = err;
    out.theta_hist = theta_hist;
    out.W_final = W_final;
    out.Hhat_hist = Hhat_hist;
    out.H_final = H_final;
    out.xhat_hist = xhat_hist;
    out.E_norm    = E_norm;
end

function dy = online_snac_ode(t, y, params)
    nx   = params.nx;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;

    % y = [ x(2); theta(nPhi*nz); xhat(2); Hhat(nPsi*nx) ]
    idx = 0;
    x = y(idx+1:idx+nx); idx = idx+nx;

    theta = y(idx+1:idx+nPhi*nz); idx = idx+nPhi*nz;

    xhat = y(idx+1:idx+nx); idx = idx+nx;

    Hhat = y(idx+1:idx+nPsi*nx); idx = idx+nPsi*nx;

    if any(~isfinite(x)) || any(~isfinite(theta)) || any(~isfinite(xhat)) || any(~isfinite(Hhat))
        error("Non-finite state at t=%.6f.", t);
    end

    W    = reshape(theta, nPhi, nz);
    Hhat = reshape(Hhat,  nPsi, nx);

    % reference, error, features
    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];

    % plant (Van der Pol) TRUE dynamics
    f_true = [x(2); (1-x(1)^2)*x(2) - x(1)];
    g      = [0; 1];

    % -----------------------
    % Drift learner: fhat(.)
    % -----------------------
    if params.nn.use_e_input
        xin = e;
    else
        xin = x;
    end
    psi = PSI_drift_approx(xin);
    fhat = (Hhat.' * psi);               % 2x1

    % -----------------------
    % Control (uses W and phi)
    % -----------------------
    % augmented dynamics z=[e;r] uses TRUE for actual z_dot in plant,
    % but sigma_hat will use fhat below.
    Fz_true = [f_true - r_dot;
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
    u_bar  = -params.cb * (params.R^-1) * (g') * gradB; % scalar

    u_nom  = u_nom + u_bar;
    u_nom  = max(min(u_nom, params.u_max), -params.u_max);

    % ---- SAFE exploration gating near boundary ----
    u_exc = u_excite(t, params.amp_mult);
    delta_h = 0.3;                       % tune
    s = min(max(h / delta_h, 0), 1); % 0..1
    if t <= params.tf_train*params.train_frac
        u = u_nom + s*u_exc;
    else
        u = u_nom;
    end
    u = max(min(u, params.u_max), -params.u_max);

    % -----------------------
    % Estimator dynamics + NN update
    % -----------------------
    xhat_dot = fhat + g*u;

    E = x - xhat;     % state estimation error
    norm_psi = 1 + (psi.'*psi);

    Hhat_dot = (params.nn.gammaH / norm_psi) * (psi * E.'); % nPsi x 2

    % freeze if weights too large
    if norm(Hhat) > params.nn.H_MAX
        Hhat_dot(:) = 0;
    end

    % clamp Hhat_dot and vectorize
    Hhat_dot = max(min(Hhat_dot, params.nn.H_DOT_MAX), -params.nn.H_DOT_MAX);
    Hhat_dot = reshape(Hhat_dot, [], 1);

    % -----------------------
    % sigma_hat (uses fhat instead of f_true)
    % -----------------------
    Fz_hat = [fhat - r_dot;
              r_dot];

    z_dot_hat = Fz_hat + Gz*u;

    % cost
    Qz = blkdiag(params.Q, zeros(params.nx));
    r_x_u = Z.'*Qz*Z + u.'*params.R*u + Bbar;

    sigma = kron(z_dot_hat, phi);

    % normalized factor (regularized)
    norm_sq  = 1 + sigma.'*sigma + params.EPS_REG;
    norm_fac = 1 / (norm_sq^2);

    % -----------------------
    % theta_dot (SNAC learning)
    % -----------------------
    if t <= params.tf_train*params.train_frac
        if norm(theta) > params.THETA_MAX
            warning("||theta||=%.2e > %.0e at t=%.4f — freezing theta_dot this step.", ...
                    norm(theta), params.THETA_MAX, t);
            theta_dot = zeros(size(theta));
        else
            bellman_err = theta.'*sigma + r_x_u;

            % base descent term
            theta_dot = -params.alpha * norm_fac * sigma * bellman_err;

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

            theta_dot = theta_dot + A_extra;
        end
    else
        theta_dot = zeros(size(theta));
    end

    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        warning("theta_dot clamped at t=%.4f (max |theta_dot|=%.2e).", ...
                t, max(abs(theta_dot)));
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    % plant derivative (TRUE plant)
    x_dot = f_true + g*u;

    dy = [x_dot; theta_dot; xhat_dot; Hhat_dot];
end

function dy = online_snac_ode_UNSAFE(t, y, params)
    nx   = params.nx;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;

    % y = [ x(2); theta(nPhi*nz); xhat(2); Hhat(nPsi*nx) ]
    idx = 0;
    x = y(idx+1:idx+nx); idx = idx+nx;

    theta = y(idx+1:idx+nPhi*nz); idx = idx+nPhi*nz;

    xhat = y(idx+1:idx+nx); idx = idx+nx;

    Hhat = y(idx+1:idx+nPsi*nx); idx = idx+nPsi*nx;

    if any(~isfinite(x)) || any(~isfinite(theta)) || any(~isfinite(xhat)) || any(~isfinite(Hhat))
        error("Non-finite state at t=%.6f.", t);
    end

    W    = reshape(theta, nPhi, nz);
    Hhat = reshape(Hhat,  nPsi, nx);

    % reference, error, features
    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];

    % plant (Van der Pol) TRUE dynamics
    f_true = [x(2); (1-x(1)^2)*x(2) - x(1)];
    g      = [0; 1];

    % -----------------------
    % Drift learner: fhat(.)
    % -----------------------
    if params.nn.use_e_input
        xin = e;
    else
        xin = x;
    end
    psi = PSI_drift_approx(xin);
    fhat = (Hhat.' * psi);               % 2x1

    % -----------------------
    % Control (uses W and phi)
    % -----------------------
    % augmented dynamics z=[e;r] uses TRUE for actual z_dot in plant,
    % but sigma_hat will use fhat below.
    Fz_true = [f_true - r_dot;
               r_dot];
    Gz = [g;
          zeros(size(g))];   % 4x1

    % nominal control from SNAC policy
    u_nom = -0.5*(params.R^-1) * (Gz') * (W') * phi;
    u_nom  = max(min(u_nom, params.u_max), -params.u_max);

    % ---- exploration ----
    u_exc = u_excite(t, params.amp_mult);

    if t <= params.tf_train*params.train_frac
        u = u_nom + u_exc;
    else
        u = u_nom;
    end
    u = max(min(u, params.u_max), -params.u_max);

    % -----------------------
    % Estimator dynamics + NN update
    % -----------------------
    xhat_dot = fhat + g*u;

    E = x - xhat;     % state estimation error
    norm_psi = 1 + (psi.'*psi);

    Hhat_dot = (params.nn.gammaH / norm_psi) * (psi * E.'); % nPsi x 2

    % freeze if weights too large
    if norm(Hhat) > params.nn.H_MAX
        Hhat_dot(:) = 0;
    end

    % clamp Hhat_dot and vectorize
    Hhat_dot = max(min(Hhat_dot, params.nn.H_DOT_MAX), -params.nn.H_DOT_MAX);
    Hhat_dot = reshape(Hhat_dot, [], 1);

    % -----------------------
    % sigma_hat (uses fhat instead of f_true)
    % -----------------------
    Fz_hat = [fhat - r_dot;
              r_dot];

    z_dot_hat = Fz_hat + Gz*u;

    % cost
    Qz = blkdiag(params.Q, zeros(params.nx));
    r_x_u = Z.'*Qz*Z + u.'*params.R*u;

    sigma = kron(z_dot_hat, phi);

    % normalized factor (regularized)
    norm_sq  = 1 + sigma.'*sigma + params.EPS_REG;
    norm_fac = 1 / (norm_sq^2);

    % -----------------------
    % theta_dot (SNAC learning)
    % -----------------------
    if t <= params.tf_train*params.train_frac
        if norm(theta) > params.THETA_MAX
            theta_dot = zeros(size(theta));
        else
            bellman_err = theta.'*sigma + r_x_u;

            % base descent term
            theta_dot = -params.alpha * norm_fac * sigma * bellman_err;

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

            theta_dot = theta_dot + A_extra;
        end
    else
        theta_dot = zeros(size(theta));
    end

    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    % plant derivative (TRUE plant)
    x_dot = f_true + g*u;

    dy = [x_dot; theta_dot; xhat_dot; Hhat_dot];
end


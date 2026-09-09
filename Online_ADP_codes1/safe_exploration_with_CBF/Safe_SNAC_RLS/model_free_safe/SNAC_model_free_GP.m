%% RLS SNAC Tracking + Safe Exploration + Drift Learning (Lin-in-Param NN) + GP Residual + SigmaHat
clear; clc; close all;
params = struct();
% --- training ---
params.tf_train   = 900;
params.amp_mult   = 3;        % exploration amplitude multiplier
params.train_frac = 0.95;     % exploration active for first 90% of tf_train
params.cb         = 0.01;   % safe exploration control
params.barrier.gamma = 6e-2;
params.barrier.c     = [-5; -5];
params.barrier.r     = 6.1;
params.barrier.clip = 0;
params.barrier.delta_h = 0.3;
% --- costs ---
params.R = 1;
params.Q = diag([100, 10]);
ode_opts = odeset('RelTol',1e-5,'AbsTol',1e-6,'MaxStep',0.1);
% --- normalization / saturation knobs ---
params.u_max     = 20;           % saturate control input
params.THETA_MAX = 1e4;          % freeze if ||theta|| too large
params.THETA_DOT_MAX = 1e4;      % clamp theta_dot
params.EPS_REG   = 1e-6;         % regularize norm factor in sigma normalization

% --- dimensions ---
params.nx = 2;
params.nR = params.nx;
params.nz = params.nx + params.nR;   % z = [e; r] => nz = 4
params.nPhi = length(PHI(ones(params.nx,1), ones(params.nx,1)));
% --- closed-loop simulation ---
params.T_sim = 30;
params.dt    = 0.01;

%% =======================
%  Drift NN params: fhat_nn(x)=Hhat' * psi(.)
%% =======================
params.nn.use_e_input = false;   % false: psi(x), true: psi(e)
params.nn.gammaH      = 20;       % learning gain for Hhat
params.nn.H_MAX       = 1e4;     % freeze if ||Hhat|| too large
params.nn.H_DOT_MAX   = 1e4;     % clamp Hhat_dot
params.nn.lambda      = 0;
params.nPsi = length(PSI_drift_approx(ones(params.nx,1)));

%% =======================
%  GP params: delta_gp(x) learns residual (f_true - fhat_nn)
%  Speed patch A+B
%% =======================
params.gp.enabled            = true;
params.gp.max_points         = 100;    % B) smaller budget
params.gp.update_every_calls = 10;     % A) update every 50 ODE calls
params.gp.ell                = [1 1];
params.gp.sf2                = 1;
params.gp.sn2                = 1e-4;
params.gp.use_e_input        = false;

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
% W_init(1,1:2) = [1, 1];
% W_init(2,1:2) = [1, 1];
theta_init = reshape(W_init, [], 1);
params.theta_length = length(theta_init); % theta length is nphi x nz
P_init = 50*eye(length(theta_init));
params.P_flat_length = params.theta_length^2;
xhat0     = zeros(params.nx,1);
Hhat0     = 0*randn(params.nPsi, params.nx); %
% Hhat0(1:3,:) = [ 0  -1 ; 1   1; 0  -1]; % H^* optimal weights 
Hhat_init = reshape(Hhat0, [], 1);

%% =======================
%  Train SAFE (barrier ON)
%% =======================
fprintf("Training SAFE (barrier ON)\n");
tic
trainSafe = run_training(theta_init, P_init, xhat0, Hhat_init, params, true, ode_opts);
safe_train_time = toc
%% =======================
%  Train UNSAFE (barrier OFF)
%% =======================
fprintf("Training UNSAFE (barrier OFF)\n");
% ode_opts = odeset('RelTol',1e-3,'AbsTol',1e-6,'MaxStep',0.1);
tic
trainUnsafe = run_training(theta_init, P_init, xhat0, Hhat_init, params, false, ode_opts);
unsafe_train_time = toc
%% =======================
%  Plot training (use the step size to reduce figure filesize)
plot_training(trainUnsafe.t',trainUnsafe.err',trainUnsafe.theta_hist',"Training Without Safety",100)
plot_training(trainSafe.t',trainSafe.err',trainSafe.theta_hist',"Training With Safety", 200)
%% =======================
%  Extra subplot diagnostics
plot_GP_drift_stuff(trainSafe,   "Drift Approx During SAFE Exploration Training", 100);
plot_GP_drift_stuff(trainUnsafe,   "Drift Approx During UNSAFE Exploration Training", 200);

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
% ylim([0 .2])
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

% saveFigures("SAFE_RLS_SNAC_MF_GP1")
%% clear super large variables
% clear trainSafe
% clear RLS_SNAC_MF
% save("RLS_SNAC_MF_GP_best.mat")
%% =====================================================================
%  Local functions
%% =====================================================================
function out = run_training(theta_init, P_init, xhat0, Hhat_init, params, UseSafety, ode_opts)
    x0 = zeros(params.nx,1);
    P_init_vec = reshape(P_init,[],1);
    % integer run id (safe for Map keys and comparisons)
    params.run_id = uint32(randi(2^32-1));
    gp_log('reset', params.run_id);

    y0 = [x0; theta_init; P_init_vec; xhat0; Hhat_init];

    if UseSafety == 1
        odefun = @(t,y) online_snac_ode(t, y, params);
    else
        odefun = @(t,y) online_snac_ode_UNSAFE(t, y, params);
    end
    [t, y] = ode23(odefun, [0 params.tf_train], y0, ode_opts);

    nx   = params.nx;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;
    theta_length = params.theta_length;
    P_flat_length = params.P_flat_length;

    x_hist           = y(:,1:nx); 
    theta_hist       = y(:,nx+1 : nx+theta_length); 
    P_flat_hist      = y(:,nx+theta_length+1: nx+theta_length + P_flat_length);
    xhat_hist        = y(:,nx+theta_length+P_flat_length+1 : nx+theta_length+P_flat_length + nx);
    Hhat_hist        = y(:,nx+theta_length+P_flat_length+nx + 1 : end);

    H_final = reshape(Hhat_hist(end,:).', nPsi, nx);

    r_hist = zeros(nx, length(t));
    P_norm = zeros(1,length(t));
    for k = 1:length(t)
        [r_hist(:,k), ~] = ref_refdot(t(k));
        P_norm(k) = norm(P_flat_hist(k,:));
    end
    err = x_hist - r_hist';

    E_hist = x_hist - xhat_hist;
    E_norm = vecnorm(E_hist, 2, 2);

    theta_final = theta_hist(end,:).';
    W_final = reshape(theta_final, nPhi, nz);

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

    out.gp_update_times = gp_log('get', params.run_id);
end

%% Safe training Function
function dy = online_snac_ode(t, y, params)
    nx   = params.nx;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;
    theta_length = params.theta_length;
    P_flat_length = params.P_flat_length;

    x           = y(1:nx); 
    theta       = y(nx+1 : nx+theta_length); 
    P_flat      = y(nx+theta_length+1: nx+theta_length + P_flat_length);
    xhat        = y(nx+theta_length+P_flat_length+1 : nx+theta_length+P_flat_length + nx);
    Hhat        = y(nx+theta_length+P_flat_length+nx + 1 : end);

    W           = reshape(theta, nPhi, nz);
    P           = reshape(P_flat,[theta_length,theta_length]);
    Hhat        = reshape(Hhat,  nPsi, nx);

    % ===== GP persistent memory (isolated per run_id) =====
    persistent gp1 gp2 run_id_last is_gp_init gp_call_counter
    if params.gp.enabled
        if isempty(is_gp_init) || ~is_gp_init || isempty(run_id_last) || (run_id_last ~= params.run_id)
            gp1 = gp_delta_init('ell', params.gp.ell, 'sf2', params.gp.sf2, ...
                                'sn2', params.gp.sn2, 'max_points', params.gp.max_points);
            gp2 = gp_delta_init('ell', params.gp.ell, 'sf2', params.gp.sf2, ...
                                'sn2', params.gp.sn2, 'max_points', params.gp.max_points);
            run_id_last = params.run_id;
            is_gp_init = true;
            gp_call_counter = 0;
        end
    end

    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];
    f_true = [x(2); (1-x(1)^2)*x(2) - x(1)];
    g      = [0; 1];

    if params.nn.use_e_input
        xin = e;
    else
        xin = x;
    end
    psi = PSI_drift_approx(xin);
    fhat_nn = (Hhat.' * psi);

    Gz = [g; zeros(size(g))];
    
    %% Controls (u_nom, u_b, u_excite)
    [Barrier, gradB, h] = B_x(x,params);
    if h <= 0 
        persistent viol_count;
        if isempty(viol_count), viol_count = 0; end
        viol_count = viol_count + 1;
        if mod(viol_count, 100) == 1
            fprintf("Safety Violation #%f: h_raw=%.4f, x=[%.3f %.3f]\n", ...
                    viol_count, h, x(1), x(2));
        end
    end
    u_nom = -0.5*(params.R^-1) * (Gz') * (W') * phi;
    u_b  = -params.cb * (params.R^-1) * (g') * gradB;
    u_nom  = max(min(u_nom + u_b, params.u_max), -params.u_max);

    u_exc = u_excite(t, params.amp_mult);
    s = min(max(h / params.barrier.delta_h, 0), 1);

    if t <= params.tf_train*params.train_frac
        u = u_nom + s*u_exc;
    else
        u = u_nom;
    end
    u = max(min(u, params.u_max), -params.u_max);

    % ===== GP update (call-based) =====
    delta_obs = f_true - fhat_nn;
    delta_gp = zeros(2,1);

    if params.gp.enabled
        gp_call_counter = gp_call_counter + 1;

        if params.gp.use_e_input
            xgp = e;
        else
            xgp = x;
        end

        if mod(gp_call_counter, params.gp.update_every_calls) == 0
            gp1 = gp_delta_update(gp1, xgp, delta_obs(1));
            gp2 = gp_delta_update(gp2, xgp, delta_obs(2));
            gp_log('add', params.run_id, t);
        end

        if ~isempty(gp1.X)
            if params.gp.use_e_input, xstar = e; else, xstar = x; end
            [mu1, ~] = gp_delta_predict(gp1, xstar);
            [mu2, ~] = gp_delta_predict(gp2, xstar);
            delta_gp = [mu1; mu2];
        end
    end

    fhat_total = fhat_nn + delta_gp;

    xhat_dot = fhat_total + g*u;

    E = x - xhat;
    norm_psi = 1 + (psi.'*psi);
    Hhat_dot = (params.nn.gammaH / norm_psi) * (psi * E.')  - params.nn.lambda*Hhat;
    if norm(Hhat) > params.nn.H_MAX, Hhat_dot(:) = 0; end
    Hhat_dot = max(min(Hhat_dot, params.nn.H_DOT_MAX), -params.nn.H_DOT_MAX);
    Hhat_dot = reshape(Hhat_dot, [], 1);

    Fz_hat = [fhat_total - r_dot; r_dot];
    z_dot_hat = Fz_hat + Gz*u;

    Qz = blkdiag(params.Q, zeros(params.nx));
    r_x_u = Z.'*Qz*Z + u.'*params.R*u + Barrier;

    sigma = kron(z_dot_hat, phi);

    norm_sq  = 1 + sigma.'*sigma + params.EPS_REG;
    norm_fac = 1 / (norm_sq^1);

    if t <= params.tf_train*params.train_frac
        if norm(theta) > params.THETA_MAX
            theta_dot = zeros(size(theta));
            P_dot = -P * (sigma*sigma')* norm_fac * P;
        else
            
            bellman_err = theta.'*sigma + r_x_u;
            Nmat  = kron(eye(params.nz), phi');       % (nz x nz*nPhi)
            Mmat  = Gz*(params.R^-1)*Gz';            % (nz x nz)
            inner = Nmat' * Mmat * Nmat;             % (nz*nPhi x nz*nPhi)

            theta_star  = params.theta_star;
            scalar_star = (theta_star.' * inner * theta_star);
            % scalar_th   = (theta.'      * inner * theta);

            F1 = 0.5 * inner * theta_star ...
                - 0.25 * sigma * norm_fac * scalar_star;                
            F2 = Nmat.' * Mmat * Nmat;       % (nz x nz*nPhi)
            
        %% new RLS stabilizing term 
            % A_extra = P*(F1 + F2*theta + 0.25*sigma*norm_fac*scalar_th); % okay results

            % A_extra = P*(F1 + F2*theta); % better results
            A_extra = P*(F1 + sigma *0.25*norm_fac *  theta.' * F2 * theta); % better results
            
            % A_extra = P*theta + .25*sigma*norm_fac*theta'*P*theta; % (bad performance)

            theta_dot = -P * norm_fac * sigma * bellman_err + A_extra;
            P_dot = -P * (sigma*sigma')* norm_fac * P;
        end
    else
        theta_dot = zeros(size(theta));
        P_dot = P*0;
        Hhat_dot = 0*Hhat_dot;
    end

    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    x_dot = f_true + g*u;

    dy = [x_dot; theta_dot; reshape(P_dot,[],1); xhat_dot; Hhat_dot];
end

%% Unsafe Training Function
function dy = online_snac_ode_UNSAFE(t, y, params)
    nx   = params.nx;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;
    theta_length = params.theta_length;
    P_flat_length = params.P_flat_length;

    x           = y(1:nx); 
    theta       = y(nx+1 : nx+theta_length); 
    P_flat      = y(nx+theta_length+1: nx+theta_length + P_flat_length);
    xhat        = y(nx+theta_length+P_flat_length+1 : nx+theta_length+P_flat_length + nx);
    Hhat        = y(nx+theta_length+P_flat_length+nx + 1 : end);

    W           = reshape(theta, nPhi, nz);
    P           = reshape(P_flat,[theta_length,theta_length]);
    Hhat        = reshape(Hhat,  nPsi, nx);

    % ===== GP persistent memory (isolated per run_id) =====
    persistent gp1 gp2 run_id_last is_gp_init gp_call_counter
    if params.gp.enabled
        if isempty(is_gp_init) || ~is_gp_init || isempty(run_id_last) || (run_id_last ~= params.run_id)
            gp1 = gp_delta_init('ell', params.gp.ell, 'sf2', params.gp.sf2, ...
                                'sn2', params.gp.sn2, 'max_points', params.gp.max_points);
            gp2 = gp_delta_init('ell', params.gp.ell, 'sf2', params.gp.sf2, ...
                                'sn2', params.gp.sn2, 'max_points', params.gp.max_points);
            run_id_last = params.run_id;
            is_gp_init = true;
            gp_call_counter = 0;
        end
    end

    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];

    f_true = [x(2); (1-x(1)^2)*x(2) - x(1)];
    g      = [0; 1];

    if params.nn.use_e_input
        xin = e;
    else
        xin = x;
    end
    psi = PSI_drift_approx(xin);
    fhat_nn = (Hhat.' * psi);

    Gz = [g; zeros(size(g))];

    u_nom = -0.5*(params.R^-1) * (Gz') * (W') * phi;
    u_nom  = max(min(u_nom, params.u_max), -params.u_max);
    u_exc = u_excite(t, params.amp_mult);

    if t <= params.tf_train * params.train_frac
        u = u_nom + u_exc;
    else
        u = u_nom;
    end
    u = max(min(u, params.u_max), -params.u_max);

    % ===== GP update (call-based) =====
    delta_obs = f_true - fhat_nn;
    delta_gp = zeros(2,1);

    if params.gp.enabled
        gp_call_counter = gp_call_counter + 1;

        if params.gp.use_e_input
            xgp = e;
        else
            xgp = x;
        end

        if mod(gp_call_counter, params.gp.update_every_calls) == 0
            gp1 = gp_delta_update(gp1, xgp, delta_obs(1));
            gp2 = gp_delta_update(gp2, xgp, delta_obs(2));
            gp_log('add', params.run_id, t);
        end

        if ~isempty(gp1.X)
            if params.gp.use_e_input, xstar = e; else, xstar = x; end
            [mu1, ~] = gp_delta_predict(gp1, xstar);
            [mu2, ~] = gp_delta_predict(gp2, xstar);
            delta_gp = [mu1; mu2];
        end
    end

    fhat_total = fhat_nn + delta_gp;

    xhat_dot = fhat_total + g*u;

    E = x - xhat;
    norm_psi = 1 + (psi.'*psi);
    Hhat_dot = (params.nn.gammaH / norm_psi) * (psi * E.')  - params.nn.lambda*Hhat;
    if norm(Hhat) > params.nn.H_MAX, Hhat_dot(:) = 0; end
    Hhat_dot = max(min(Hhat_dot, params.nn.H_DOT_MAX), -params.nn.H_DOT_MAX);
    Hhat_dot = reshape(Hhat_dot, [], 1);

    Fz_hat = [fhat_total - r_dot; r_dot];
    z_dot_hat = Fz_hat + Gz*u;

    Qz = blkdiag(params.Q, zeros(params.nx));
    r_x_u = Z.'*Qz*Z + u.'*params.R*u;

    sigma = kron(z_dot_hat, phi);

    norm_sq  = 1 + sigma.'*sigma + params.EPS_REG;
    norm_fac = 1 / (norm_sq^2);

    if t <= params.tf_train*params.train_frac
        if norm(theta) > params.THETA_MAX
            theta_dot = zeros(size(theta));
            P_dot = -P * (sigma*sigma')* norm_fac * P;
        else
            
            bellman_err = theta.'*sigma + r_x_u;
            Nmat  = kron(eye(params.nz), phi');       % (nz x nz*nPhi)
            Mmat  = Gz*(params.R^-1)*Gz';            % (nz x nz)
            inner = Nmat' * Mmat * Nmat;             % (nz*nPhi x nz*nPhi)

            theta_star  = params.theta_star;
            scalar_star = (theta_star.' * inner * theta_star);
            scalar_th   = (theta.'      * inner * theta);

            F1 = 0.5 * inner * theta_star ...
                - 0.25 * sigma * norm_fac * scalar_star;                
            F2 = Nmat.' * Mmat * Nmat;       % (nz x nz*nPhi)
            
        %% new RLS stabilizing term 
            % A_extra = P*(F1 + F2*theta + 0.25*sigma*norm_fac*scalar_th); % okay results

            % A_extra = P*(F1 + F2*theta); % better results
            A_extra = P*(F1 + sigma *0.25*norm_fac *  theta.' * F2 * theta); % better results
            
            % A_extra = P*theta + .25*sigma*norm_fac*theta'*P*theta; % (bad performance)

            theta_dot = -P * norm_fac * sigma * bellman_err + A_extra;
            P_dot = -P * (sigma*sigma')* norm_fac * P;
        end
    else
        theta_dot = zeros(size(theta));
        P_dot = P*0;
        Hhat_dot = 0*Hhat_dot;
    end

    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    x_dot = f_true + g*u;

    dy = [x_dot; theta_dot; reshape(P_dot,[],1); xhat_dot; Hhat_dot];
end

%% =======================
%  GP update-time logger (per run_id) using containers.Map
%% =======================
function out = gp_log(cmd, run_id, t)
    persistent MAP
    if isempty(MAP)
        MAP = containers.Map('KeyType','uint32','ValueType','any');
    end

    switch lower(cmd)
        case 'reset'
            MAP(run_id) = zeros(0,1);
            out = [];
        case 'add'
            if ~isKey(MAP, run_id)
                MAP(run_id) = zeros(0,1);
            end
            MAP(run_id) = [MAP(run_id); t];
            out = [];
        case 'get'
            if isKey(MAP, run_id)
                out = MAP(run_id);
            else
                out = zeros(0,1);
            end
        otherwise
            error('gp_log: unknown cmd "%s"', cmd);
    end
end

%% =======================
%  Lightweight GP functions (scalar output)
%% =======================
function gp = gp_delta_init(varargin)
    p = inputParser;
    addParameter(p, 'ell', 1);
    addParameter(p, 'sf2', 1);
    addParameter(p, 'sn2', 1e-4);
    addParameter(p, 'max_points', 100);
    parse(p, varargin{:});

    gp.ell = p.Results.ell;
    gp.sf2 = p.Results.sf2;
    gp.sn2 = p.Results.sn2;
    gp.max_points = p.Results.max_points;

    gp.X = zeros(0,2);
    gp.y = zeros(0,1);
    gp.L = [];
    gp.alpha = [];
end

function gp = gp_delta_update(gp, x, y)
    x = x(:).';
    y = double(y);

    gp.X = [gp.X; x];
    gp.y = [gp.y; y];

    if size(gp.X,1) > gp.max_points
        k = size(gp.X,1) - gp.max_points;
        gp.X(1:k,:) = [];
        gp.y(1:k,:) = [];
    end

    K = gp_rbf(gp.X, gp.X, gp.ell, gp.sf2);
    n = size(K,1);
    K = K + (gp.sn2 + 1e-12) * eye(n);

    jitter = 1e-10;
    for tries = 1:8
        [L,flag] = chol(K,'lower');
        if flag==0
            gp.L = L;
            break;
        end
        K = K + jitter*eye(n);
        jitter = jitter*10;
    end
    if flag~=0
        error('GP Cholesky failed.');
    end

    gp.alpha = gp.L' \ (gp.L \ gp.y);
end

function [mu, s2] = gp_delta_predict(gp, xstar)
    xstar = xstar(:).';

    if isempty(gp.X)
        mu = 0;
        s2 = gp.sf2;
        return;
    end

    kstar = gp_rbf(gp.X, xstar, gp.ell, gp.sf2);
    mu = kstar.' * gp.alpha;

    v  = gp.L \ kstar;
    kss = gp_rbf(xstar, xstar, gp.ell, gp.sf2);
    s2 = max(0, kss - v.'*v);
end

function K = gp_rbf(X1, X2, ell, sf2)
    if isscalar(ell)
        ell = ell * ones(1,size(X1,2));
    end
    X1s = X1 ./ ell;
    X2s = X2 ./ ell;

    dist2 = sum(X1s.^2,2) - 2*(X1s*X2s.') + sum(X2s.^2,2).';
    K = sf2 * exp(-0.5*dist2);
end
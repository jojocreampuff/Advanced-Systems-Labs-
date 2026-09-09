%% SNAC Tracking RLS Tracking Without safety
clear; clc; close all;
params = struct();
% --- training ---
params.tf   = 1000;
params.amp_mult   = .3;         % exploration amplitude multiplier
params.amp_mult_sq = .35 ;
params.train_frac = 0.90;      
% --- costs ---
params.R = .1*diag([1, 1, 1]);
params.Q = diag([2, 2, 2, 1, 1, 1]);

% ----- ODE solver tol -------
ode_opts = odeset('RelTol',10e-3,'AbsTol',10e-6,'MaxStep',0.1);

% --- dimensions ---
params.nx = 6;
params.nxQ = 7;
params.nR = params.nx;
params.nz = params.nx + params.nR;   % z = [e; r] => nz = 12
params.nPhi = length(PHI(ones(params.nx,1), ones(params.nx,1)));

% --- normalization / saturation knobs ---
params.u_max     = 2;           % saturate control input
params.THETA_MAX = 1e4;          % freeze if ||theta|| too large
params.THETA_DOT_MAX = 1e4;      % clamp theta_dot
% params.EPS_REG   = 1e-6;         % regularize norm factor in sigma normalization

% --- linear model used by CARE target ---
params.Ix = 0.2;
params.Iy = 0.3;
params.Iz = 0.4;

%% =======================
%  Drift NN params: fhat_nn(x)=Hhat' * psi(.)
%% =======================
params.nn.use_e_input = false;   % false: psi(x), true: psi(e)
params.nn.gammaH      = 10;       % learning gain for Hhat
params.nn.H_MAX       = 1e4;     % freeze if ||Hhat|| too large
params.nn.H_DOT_MAX   = 1e4;     % clamp Hhat_dot
params.nn.lambda      = 0;
params.nPsi = length(PSI_drift_approx(ones(params.nx,1)));

%% =======================
%  GP params: delta_gp(x) learns residual (f_true - fhat_nn)
%% =======================
params.gp.enabled            = true;
params.gp.max_points         = 100;   
params.gp.update_every_calls = 10;   
params.gp.ell                = ones(1, params.nx);
params.gp.sf2                = 1;
params.gp.sn2                = 1e-4;
params.gp.use_e_input        = false;
%% LQR comparison

A_att = [0  0   0   1   0   0   
        0   0   0   0   1   0   
        0   0   0   0   0   1   
        0   0   0   0   0   0  
        0   0   0   0   0   0  
        0   0   0   0   0   0];

B_att = [0   0   0 
         0   0   0 
         0   0   0 
         1/params.Ix    0   0
         0   1/params.Iy    0
         0    0      1/params.Iz];

[~, P_lqr, ~]= lqr(A_att,B_att,params.Q,params.R);

%% theta star guess 
W_star = zeros(params.nPhi, params.nz);
W_star(1:6,1:6) = P_lqr(1:6,1:6);
params.theta_star = reshape(W_star, [], 1);

% --- initial weights ---
W_init = zeros(params.nPhi, params.nz);
W_init(1:6,1:6) = ones(params.nx,params.nx);
theta_init = reshape(W_init, [], 1);
params.theta_length = length(theta_init); % theta length is nphi x nz
P_init = 1*eye(length(theta_init));
params.P_flat_length = params.theta_length^2;
xhat0     = zeros(params.nx,1);
Hhat0     = 0*randn(params.nPsi, params.nx); %
Hhat_init = reshape(Hhat0, [], 1);
% --- closed-loop simulation ---
params.T_sim = 20;
params.dt    = 0.01;

%%  Train RLS SNAC model FREE
fprintf("Training UNSAFE (barrier OFF)\n");
tic
RLS_SNAC_MF = run_training(theta_init, P_init, xhat0, Hhat_init, params, ode_opts);
unsafe_train_time = toc
%%  Plot training (use the step size to reduce figure filesize)
plot_training(RLS_SNAC_MF.t',RLS_SNAC_MF.err,RLS_SNAC_MF.theta_hist',RLS_SNAC_MF.P_norm',"RLS Training",10)
plot_GP_drift_stuff(RLS_SNAC_MF,   "Drift Approx  and GP", 200);
%%  Run time: Closed-loop sim (ode45)
RLS_SNAC_tracking_sim  = simulate_closed_loop(RLS_SNAC_MF.W_final, params);

fprintf("\n=== Closed-loop Tracking RMSE ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.RMSE);

fprintf("\n=== Closed-loop Tracking Control Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.control_cost);

fprintf("\n=== Closed-loop Tracking Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.tracking_cost);

%% =======================
%  Plots
% =======================
figure; hold on; grid on; box on;
plot3(RLS_SNAC_tracking_sim.x(1,:),   RLS_SNAC_tracking_sim.x(2,:), RLS_SNAC_tracking_sim.x(3,:), 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:500:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
plot3(RLS_SNAC_tracking_sim.ref(1,:), RLS_SNAC_tracking_sim.ref(2,:), RLS_SNAC_tracking_sim.ref(3,:),"Color","g","LineStyle","-","LineWidth",1);
xlabel("$\phi$", Interpreter="latex"); ylabel("$\theta$", Interpreter="latex"); zlabel("$\psi$", Interpreter="latex");
% xlim([-1.5 1.5])
% ylim([-2 2])
legend("RLS SNAC","Reference",'Location','best', Interpreter="latex");
title("State-space tracking");

figure; hold on; grid on; box on;
plot(RLS_SNAC_tracking_sim.time,   RLS_SNAC_tracking_sim.L2_err, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("error");
% ylim([0 .2])
title("Error plot");

figure; hold on; grid on; box on;
plot(RLS_SNAC_tracking_sim.time,   RLS_SNAC_tracking_sim.u, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("Control");
title("Control Plot");


% saveFigures("SNAC_RLS_pos_best_so_far")
% save("SNAC_RLS_att_3")
%% =======================
%  Local functions
%% =======================
function out = run_training(theta_init, P_init, xhat0, Hhat_init, params,  ode_opts)
    P_init_vec = reshape(P_init,[],1);
    xQ0 = [1;0;0;0;0;0;0]; % quaterion equalibrium
    y0 = [xQ0; theta_init; P_init_vec; xhat0; Hhat_init];
    % integer run id (safe for Map keys and comparisons)
    params.run_id = uint32(randi(2^32-1));
    gp_log('reset', params.run_id);
    
    odefun = @(t,y) online_snac_ode_unsafe(t, y, params);

    [t, y] = ode23(odefun, [0 params.tf], y0, ode_opts);

    nx   = params.nx;
    nxQ  = params.nxQ;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;
    theta_length = params.theta_length;
    P_flat_length = params.P_flat_length;

    xQ_hist          = y(:,1:nxQ); 
    theta_hist       = y(:,nxQ+1 : nxQ+theta_length); 
    P_flat_hist      = y(:,nxQ+theta_length+1: nxQ+theta_length + P_flat_length);
    xhat_hist        = y(:,nxQ+theta_length+P_flat_length+1 : nxQ+theta_length+P_flat_length + nx);
    Hhat_hist        = y(:,nxQ+theta_length+P_flat_length+nx + 1 : end);

    H_final = reshape(Hhat_hist(end,:).', nPsi, nx);

    r_hist = zeros(nx, length(t));
    P_norm = zeros(1,length(t));
    for k = 1:length(t)
        [r_hist(:,k), ~] = ref_refdot(t(k));
        P_norm(k) = norm(P_flat_hist(k,:));
        x_hist(:,k) = Quat2Euler(xQ_hist(k,:));
    end
    err = x_hist - r_hist;

    E_hist = x_hist' - xhat_hist;
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
    out.P_norm = P_norm;
    out.gp_update_times = gp_log('get', params.run_id);
end

function dy = online_snac_ode_unsafe(t, y, params)
    nx   = params.nx;
    nxQ  = params.nxQ;
    nPhi = params.nPhi;
    nz   = params.nz;
    nPsi = params.nPsi;
    theta_length = params.theta_length;
    P_flat_length = params.P_flat_length;

    xQ           = y(1:nxQ); 
    theta       = y(nxQ+1 : nxQ+theta_length); 
    P_flat      = y(nxQ+theta_length+1: nxQ+theta_length + P_flat_length);
    xhat        = y(nxQ+theta_length+P_flat_length+1 : nxQ+theta_length+P_flat_length + nx);
    Hhat        = y(nxQ+theta_length+P_flat_length+nx + 1 : end);

    W           = reshape(theta, nPhi, nz);
    P           = reshape(P_flat,[theta_length,theta_length]);
    Hhat        = reshape(Hhat,  nPsi, nx);

    x = Quat2Euler(xQ); % turn global quat into euler

    if any(~isfinite(x)) || any(~isfinite(theta))
        error("Non-finite state/theta at t=%.6f.", t);
    end

    % ===== GP persistent memory (isolated per run_id) =====
    persistent gps run_id_last is_gp_init gp_call_counter
    if params.gp.enabled
        if isempty(is_gp_init) || ~is_gp_init || isempty(run_id_last) || (run_id_last ~= params.run_id)
            gps = cell(params.nx,1);
            for i = 1:params.nx
                gps{i} = gp_delta_init( ...
                    'ell', params.gp.ell, ...
                    'sf2', params.gp.sf2, ...
                    'sn2', params.gp.sn2, ...
                    'max_points', params.gp.max_points, ...
                    'input_dim', params.nx);
            end
            run_id_last = params.run_id;
            is_gp_init = true;
            gp_call_counter = 0;
        end
    end
    % reference, error, features
    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];

    % plant (Van der Pol)
    f_true = att_F(x,params.Ix, params.Iy, params.Iz);
    g = [0 0 0; 0 0 0; 0 0 0; 1/params.Ix 0 0; 0 1/params.Iy 0; 0 0 1/params.Iz];
    if abs(x(2)) >= 1.3963 % if this happens, training is usually ruined
        fprintf("close to singularity @ t = %f s, x2 = %f deg\n",t,rad2deg(x(2)))
        fprintf("f(3) is %f, clipping to: +-1.6\n",f_true(3))
    end
    max1 = 1.6; % this number was found through testing, this is where the dynamics start to blow up
    if f_true(3) >= max1
        f_true(3) = max1;
    elseif f_true(3) <= -max1
        f_true(3) = -max1;
    end
    
    Gz = [g;
          zeros(size(g))]; 

    if params.nn.use_e_input
        xin = e;
    else
        xin = x;
    end
    psi = PSI_drift_approx(xin);
    fhat_nn = (Hhat.' * psi);

    % nominal control from SNAC policy
    u = -0.5*(params.R^-1) * (Gz') * (W') * phi;
    u  = max(min(u, params.u_max), -params.u_max);
    u_exc = u_excite(t, params.amp_mult_sq) + u_excite_sq(t, params.amp_mult);

    if t <= params.tf*params.train_frac
        u = u + u_exc;
    end
    u = max(min(u, params.u_max), -params.u_max);
    % ===== GP update (call-based) =====
    delta_obs = f_true - fhat_nn;
    delta_gp = zeros(nx,1);

    if params.gp.enabled
        gp_call_counter = gp_call_counter + 1;
    
        if params.gp.use_e_input
            xgp = e;
        else
            xgp = x;
        end
    
        if mod(gp_call_counter, params.gp.update_every_calls) == 0
            for i = 1:nx
                gps{i} = gp_delta_update(gps{i}, xgp, delta_obs(i));
            end
            gp_log('add', params.run_id, t);
        end
    
        if ~isempty(gps{1}.X)
            if params.gp.use_e_input
                xstar = e;
            else
                xstar = x;
            end
    
            for i = 1:nx
                [mui, ~] = gp_delta_predict(gps{i}, xstar);
                delta_gp(i) = mui;
            end
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

    % normalized factor (regularized)
    norm_sq  = 1 + sigma.'*sigma;% + params.EPS_REG;
    norm_fac = 1 / (norm_sq^1);

    % -------- theta_dot --------
    if t <= params.tf*params.train_frac
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

            % F1 = 0.5 * inner * theta_star ...
            %    - 0.5 * sigma * norm_fac * scalar_star ...
            %    + 0.25 * sigma * norm_fac * scalar_star ...
            %    - norm_fac * (sigma*sigma') * theta_star;
            % 
            % F2 = norm_fac * (sigma*sigma');

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
        % theta_dot = zeros(size(theta));
        % P_dot = P*0;
        % Hhat_dot = 0*Hhat_dot;
        theta_dot = -P * norm_fac * sigma * bellman_err + A_extra;
        P_dot = -P * (sigma*sigma')* norm_fac * P;
    end

    % clamp theta_dot
    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    % plant derivative
    fq = Quaterion_f(xQ,params.Ix, params.Iy, params.Iz);
    gq =[0 0 0; 0 0 0; 0 0 0; 0 0 0; 1/params.Ix 0 0; 0 1/params.Iy 0; 0 0 1/params.Iz];
    xQ_dot = fq + gq*u;

    dy = [xQ_dot; theta_dot; reshape(P_dot,[],1); xhat_dot; Hhat_dot];
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
    addParameter(p, 'input_dim', 1);
    parse(p, varargin{:});

    gp.ell = p.Results.ell;
    gp.sf2 = p.Results.sf2;
    gp.sn2 = p.Results.sn2;
    gp.max_points = p.Results.max_points;
    gp.input_dim = p.Results.input_dim;

    gp.X = zeros(0, gp.input_dim);
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
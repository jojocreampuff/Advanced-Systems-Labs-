%% SNAC Tracking RLS Tracking Without safety
clear; clc; close all;
%% =======================
%  Params / Setup
% =======================
params = struct();
% --- training ---
params.tf   = 900;
params.amp_mult   = 3;         % exploration amplitude multiplier
params.train_frac = 0.90;      % exploration active for first 90% of tf
% --- costs ---
params.R = 1;
params.Q = diag([100, 5]);

% ----- ODE solver tol -------
ode_opts = odeset('RelTol',1e-3,'AbsTol',1e-6,'MaxStep',0.1);

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
params.theta_length = length(theta_init); % theta length is nphi x nz
P_init = 50*eye(length(theta_init));
% --- closed-loop simulation ---
params.T_sim = 20;
params.dt    = 0.01;

%% RLS SNAC Tracking
fprintf("RLS SNAC Tracking...\n");
tic
trainUnsafe = run_training(theta_init, P_init, params, ode_opts);
safe_train_time = toc

%%  Plot training (use the step size to reduce figure filesize)
plot_training(trainUnsafe.t',trainUnsafe.err',trainUnsafe.theta_hist',trainUnsafe.P_norm',"RLS Tracking", 1)

%%  Run time: Closed-loop sim (ode45)
RLS_SNAC_tracking_sim   = simulate_closed_loop(trainUnsafe.W_final,   params, false);

fprintf("\n=== Closed-loop Tracking RMSE ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.RMSE);

fprintf("\n=== Closed-loop Tracking Control Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.control_cost);

fprintf("\n=== Closed-loop Tracking Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", RLS_SNAC_tracking_sim.tracking_cost);

%% =======================
%  Plots
% =======================
figure; hold on; grid on;
plot(RLS_SNAC_tracking_sim.x(1,:),   RLS_SNAC_tracking_sim.x(2,:), 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:500:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
plot(RLS_SNAC_tracking_sim.ref(1,:), RLS_SNAC_tracking_sim.ref(2,:),"Color","g","LineStyle","-","LineWidth",1);
xlabel("x_1"); ylabel("x_2");
xlim([-1.5 1.5])
ylim([-2 2])
legend("RLS SNAC","Reference",'Location','best');
title("State-space tracking");

figure; hold on; grid on;
plot(RLS_SNAC_tracking_sim.time,   RLS_SNAC_tracking_sim.L2_err, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("error");
ylim([0 .2])
title("Error plot");

figure; hold on; grid on;
plot(RLS_SNAC_tracking_sim.time,   RLS_SNAC_tracking_sim.u, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:70:length(RLS_SNAC_tracking_sim.time), 'MarkerSize', 5);
xlabel("Time (s)"); ylabel("Control");
title("Control Plot");

% saveFigures("SNAC_RLS_results1")
% save("SNAC_RLS_results1")
%% =======================
%  Local functions
%% =======================
function out = run_training(theta_init, P_init, params, ode_opts)
    P_init_vec = reshape(P_init,[],1);
    x0 = zeros(params.nx,1);
    y0 = [x0; theta_init; P_init_vec];
    
    odefun = @(t,y) online_snac_ode_unsafe(t, y, params);

    [t, y] = ode23(odefun, [0 params.tf], y0, ode_opts);

    x_hist     = y(:, 1:params.nx);
    theta_hist = y(:, params.nx+1 : params.theta_length + params.nx);
    P_hist_vec = y(:,params.theta_length + params.nx + 1:end)';


    r_hist = zeros(params.nx, length(t));
    P_norm = zeros(1,length(t));
    for k = 1:length(t)
        [r_hist(:,k), ~] = ref_refdot(t(k));
        P_norm(k) = norm(P_hist_vec(:,k));
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
    out.P_norm = P_norm;
end

function dy = online_snac_ode_unsafe(t, y, params)
    x     = y(1:params.nx);
    theta = y(params.nx+1 : params.nx + params.theta_length);
    P_flat     = y(params.nx+1 + params.theta_length: end);
    P       =reshape(P_flat,[(params.theta_length),(params.theta_length)]);

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
    
    u  = max(min(u, params.u_max), -params.u_max);
    u_exc = u_excite(t, params.amp_mult);

    if t <= params.tf*params.train_frac
        u = u + u_exc;
    end
    u = max(min(u, params.u_max), -params.u_max);

    z_dot = Fz + Gz*u;

    Qz = blkdiag(params.Q, zeros(params.nx));

    r_x_u = Z.'*Qz*Z + u.'*params.R*u;

    sigma = kron(z_dot, phi);

    % normalized factor (regularized)
    norm_sq  = 1 + sigma.'*sigma + params.EPS_REG;
    norm_fac = 1 / (norm_sq^2);

    % -------- theta_dot --------
    if t <= params.tf*params.train_frac
        if norm(theta) > params.THETA_MAX
            warning("||theta||=%.2e > %.0e at t=%.4f — freezing theta_dot this step.", ...
                    norm(theta), params.THETA_MAX, t);
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
        theta_dot = zeros(size(theta));
        P_dot = P*0;
    end

    % clamp theta_dot
    if any(abs(theta_dot) > params.THETA_DOT_MAX)
        warning("theta_dot clamped at t=%.4f (max |theta_dot|=%.2e).", ...
                t, max(abs(theta_dot)));
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    % plant derivative
    x_dot = f_vec + g*u;

    dy = [x_dot; theta_dot; reshape(P_dot,[],1)];
end




%% SNAC Tracking with GD of quadcopter attitude controller
clear; clc; close all;
params = struct();
% --- training ---
params.tf   = 1000;
params.amp_mult   = .35;         % exploration amplitude multiplier
params.alpha = 10;
params.amp_mult_sq = .35 ;
params.train_frac = 0.90;      
% --- costs ---
params.R = .1*diag([1, 1, 1]);
params.Q = diag([2, 2, 1, 1, 1, 1]);

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
params.EPS_REG   = 1e-6;         % regularize norm factor in sigma normalization

% --- linear model used by CARE target ---
params.Ix = 0.2;
params.Iy = 0.3;
params.Iz = 0.4;

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
W_star(1:6,1:6) = 2*P_lqr(1:6,1:6);
params.theta_star = reshape(W_star, [], 1);

% --- initial weights ---
W_init = zeros(params.nPhi, params.nz);
W_init(1:6,1:6) = ones(params.nx,params.nx);
theta_init = reshape(W_init, [], 1);
params.theta_length = length(theta_init); % theta length is nphi x nz
% --- closed-loop simulation ---
params.T_sim = 20;
params.dt    = 0.01;

%%  Train GD SNAC model based
fprintf("Training\n");
tic
GD_SNAC_MF = run_training(theta_init, params, ode_opts);
unsafe_train_time = toc
%%  Plot training (use the step size to reduce figure filesize)
plot_training(GD_SNAC_MF.t',GD_SNAC_MF.err,GD_SNAC_MF.theta_hist',"RLS Training",10)
%%  Run time: Closed-loop sim (ode45)
GD_SNAC_tracking_sim  = simulate_closed_loop(GD_SNAC_MF.W_final, params);

fprintf("\n=== Closed-loop Tracking RMSE ===\n");
fprintf("  RLS Tracking: %.6f\n", GD_SNAC_tracking_sim.RMSE);

fprintf("\n=== Closed-loop Tracking Control Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", GD_SNAC_tracking_sim.control_cost);

fprintf("\n=== Closed-loop Tracking Cost ===\n");
fprintf("  RLS Tracking: %.6f\n", GD_SNAC_tracking_sim.tracking_cost);

%% =======================
%  Plots
% =======================
figure; hold on; grid on; box on;
plot3(GD_SNAC_tracking_sim.x(1,:),   GD_SNAC_tracking_sim.x(2,:), GD_SNAC_tracking_sim.x(3,:), 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
     'MarkerIndices', 1:500:length(GD_SNAC_tracking_sim.time), 'MarkerSize', 5);
plot3(GD_SNAC_tracking_sim.ref(1,:), GD_SNAC_tracking_sim.ref(2,:), GD_SNAC_tracking_sim.ref(3,:),"Color","g","LineStyle","-","LineWidth",1);
xlabel("$\phi$", Interpreter="latex"); ylabel("$\theta$", Interpreter="latex"); zlabel("$\psi$", Interpreter="latex");
% xlim([-1.5 1.5])
% ylim([-2 2])
legend("RLS SNAC","Reference",'Location','best', Interpreter="latex");
title("State-space tracking");
% 
% figure; hold on; grid on; box on;
% plot(GD_SNAC_tracking_sim.time,   GD_SNAC_tracking_sim.L2_err, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
%      'MarkerIndices', 1:70:length(GD_SNAC_tracking_sim.time), 'MarkerSize', 5);
% xlabel("Time (s)"); ylabel("error");
% % ylim([0 .2])
% title("Error plot");
% 
% figure; hold on; grid on; box on;
% plot(GD_SNAC_tracking_sim.time,   GD_SNAC_tracking_sim.u, 'Color', [0 0 1],"LineStyle","--", 'Marker', 'o', 'LineWidth', 1, ...
%      'MarkerIndices', 1:70:length(GD_SNAC_tracking_sim.time), 'MarkerSize', 5);
% xlabel("Time (s)"); ylabel("Control");
% title("Control Plot");


% saveFigures("SNAC_RLS_pos_best_so_far")
save("SNAC_GD_att_2")
%% =======================
%  Local functions
%% =======================
function out = run_training(theta_init, params,  ode_opts)
    xQ0 = [1;0;0;0;0;0;0]; % quaterion equalibrium
    y0 = [xQ0; theta_init;];
    
    odefun = @(t,y) online_snac_ode_unsafe(t, y, params);

    [t, y] = ode23(odefun, [0 params.tf], y0, ode_opts);

    nx   = params.nx;
    nxQ  = params.nxQ;
    nPhi = params.nPhi;
    nz   = params.nz;
    theta_length = params.theta_length;

    xQ_hist          = y(:,1:nxQ); 
    theta_hist       = y(:,nxQ+1 : nxQ+theta_length); 

    r_hist = zeros(nx, length(t));
    for k = 1:length(t)
        [r_hist(:,k), ~] = ref_refdot(t(k));
        x_hist(:,k) = Quat2Euler(xQ_hist(k,:));
    end
    err = x_hist - r_hist;

    theta_final = theta_hist(end,:).';
    W_final = reshape(theta_final, nPhi, nz);

    out = struct();
    out.t = t;
    out.x_hist = x_hist;
    out.r_hist = r_hist;
    out.err = err;
    out.theta_hist = theta_hist;
    out.W_final = W_final;

end

function dy = online_snac_ode_unsafe(t, y, params)
    nx   = params.nx;
    nxQ  = params.nxQ;
    nPhi = params.nPhi;
    nz   = params.nz;
    theta_length = params.theta_length;

    xQ           = y(1:nxQ); 
    theta       = y(nxQ+1 : nxQ+theta_length); 

    W           = reshape(theta, nPhi, nz);

    x = Quat2Euler(xQ); % turn global quat into euler

    if any(~isfinite(x)) || any(~isfinite(theta))
        error("Non-finite state/theta at t=%.6f.", t);
    end

    % reference, error, features
    [r, r_dot] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Z   = [e; r];

    % plan
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

    % nominal control from SNAC policy
    u = -0.5*(params.R^-1) * (Gz') * (W') * phi;
    u  = max(min(u, params.u_max), -params.u_max);
    u_exc = u_excite(t, params.amp_mult_sq) + u_excite_sq(t, params.amp_mult);

    if t <= params.tf*params.train_frac
        u = u + u_exc;
    end
    u = max(min(u, params.u_max), -params.u_max);

    Fz = [f_true - r_dot; r_dot];
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
        theta_dot = max(min(theta_dot, params.THETA_DOT_MAX), -params.THETA_DOT_MAX);
    end

    % plant derivative
    fq = Quaterion_f(xQ,params.Ix, params.Iy, params.Iz);
    gq =[0 0 0; 0 0 0; 0 0 0; 0 0 0; 1/params.Ix 0 0; 0 1/params.Iy 0; 0 0 1/params.Iz];
    xQ_dot = fq + gq*u;

    dy = [xQ_dot; theta_dot];
end


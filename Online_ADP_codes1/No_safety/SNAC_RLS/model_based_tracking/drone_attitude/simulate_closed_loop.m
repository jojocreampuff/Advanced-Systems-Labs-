function sim = simulate_closed_loop(W, params, ub_logic)
% ub_logic added or removes ub at runtime 
    % real dyanamics
    % f_van = @(x) [x(2); (1-x(1)^2)*x(2) - x(1)];
    % g     = [0; 1];
    f = @(x) [x(4); x(5); x(6); 0; 0; 0];
    g = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];

    odefun = @(t,x) f(x) + g*compute_u(t, x, W, g, params, ub_logic);
    tspan  = [0, params.T_sim];
    x0     = .2*randn(params.nx,1);

    ode_opts_sim = odeset('RelTol',1e-6,'AbsTol',1e-7,'MaxStep',params.dt);
    [t_ode, x_ode] = ode45(odefun, tspan, x0, ode_opts_sim);

    time = (params.dt:params.dt:params.T_sim);
    N = length(time);

    x_out = interp1(t_ode, x_ode, time)';   % (nx, N)
    ref   = zeros(params.nx, N);
    u_out = zeros(3, N);
    % h_log = zeros(1, N);
    err   = zeros(params.nx, N);

    for i = 1:N
        [ref(:,i), ~] = ref_refdot(time(i));
        u_out(:,i) = compute_u(time(i), x_out(:,i), W, g, params, ub_logic);
        % [~, ~, h_log(i)] = B_x(x_out(:,i),params);
        err(:,i) = x_out(:,i) - ref(:,i);
    end

    RMSE = sqrt(mean(err(:).^2));
    L2_err = (err(1,:).^2 + err(2,:).^2).^.5;
    % safety_viol = sum(h_log < 0);
    tracking_cost = params.dt * sum( sum( err .* (params.Q*err), 1 ) );
    control_cost = params.dt * sum( sum( u_out .* (params.R*u_out), 1 ) );
    cost = tracking_cost + control_cost;
    
    sim = struct();
    sim.time = time;
    sim.x = x_out;
    sim.u = u_out;
    sim.ref = ref;
    % sim.h_log = h_log;
    sim.err = err;
    sim.RMSE = RMSE;
    sim.L2_err = L2_err;
    % sim.safety_viol = safety_viol;
    sim.tracking_cost = tracking_cost;
    sim.control_cost = control_cost;
    sim.cost = cost;
end

function u = compute_u(t, x, W, g, params, ub_logic)
    [r, ~] = ref_refdot(t);
    e   = x - r;
    phi = PHI(e, r);
    Gz  = [g; zeros(size(g))];  % 4x1
    u_nom   = -0.5*(params.R^-1) * (Gz') * (W') * phi;

    % [B, gradB, h] = B_x(x,params);
    % u_bar  = -params.cb * (params.R^-1) * (g') * gradB;        % scalar
    % u  = u_nom + ub_logic*u_bar;
    u = u_nom;
    % saturate
    u = max(min(u, params.u_max), -params.u_max);
end
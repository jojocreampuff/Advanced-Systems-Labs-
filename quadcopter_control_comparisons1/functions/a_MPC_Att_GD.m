function tau_guess = a_MPC_Att_GD(att_ref_horizon, X_current, tau_guess, Ha, Q, R, dt, Ix, Iy, Iz, att_iter, alpha, decay_rate, eps_jac)
% Nonlinear attitude MPC using gradient descent with adjoint gradient.
%
% State: x = [phi; theta; psi; p; q; r] (6x1)
% Input: u = [tau_x; tau_y; tau_z]      (3x1)
%
% att_ref_horizon: 6 x (Ha+1) desired trajectory over horizon
% X_current:       6 x 1 current state
% tau_guess:       3 x Ha initial guess (warm-started recommended)
% Q (6x6), R (3x3): weights (SPD, often diagonal)
% dt: timestep
% Ix,Iy,Iz: inertias
% att_iter: GD iterations
% alpha: base step size
% decay_rate: learning rate decay factor
%
% Outputs:
% tau_guess: optimized torque sequence (3xHa). Apply tau_guess(:,1).
% J_current: scalar cost for the final iteration.
Q = Q/dt;
R = R/dt;
% ---- dynamics (your provided) ----
f_att = @(x) [(x(4) + x(5)*(sin(x(1))*tan(x(2))) + x(6)*(cos(x(1))*tan(x(2))));
              (x(5)*cos(x(1)) - x(6)*sin(x(1)));
              (x(5)*sin(x(1))/cos(x(2)) + x(6)*cos(x(1))/cos(x(2)));
              ((Iy - Iz) / Ix * x(5) * x(6));
              ((Iz - Ix) / Iy * x(4) * x(6));
              ((Ix - Iy) / Iz * x(4) * x(5))];

g_att = [0 0 0;
         0 0 0;
         0 0 0;
         1/Ix 0 0;
         0 1/Iy 0;
         0 0 1/Iz];

% Discrete step
step = @(x,u) x + dt*(f_att(x) + g_att*u);


% Ensure horizon size is Ha+1
if size(att_ref_horizon,2) < Ha+1
    last = att_ref_horizon(:,end);
    att_ref_horizon(:,end+1:Ha+1) = repmat(last,1,Ha+1-size(att_ref_horizon,2));
end

% Optional torque clamp (helps stability while debugging)
tau_max = [10;10;10]; % <-- adjust to your vehicle
clamp_tau = @(u) max(min(u, tau_max), -tau_max);

for it = 1:att_iter

    % ----- Forward rollout -----
    X = zeros(6, Ha+1);
    X(:,1) = X_current(:);

    % store df/dx along trajectory (for adjoint)
    A = zeros(6,6,Ha);   % A_k = I + dt*dfdx(x_k)

    for k = 1:Ha
        xk = X(:,k);
        uk = tau_guess(:,k);

        % propagate
        X(:,k+1) = step(xk, uk);

        % compute Jacobian df/dx numerically at xk
        fx0 = f_att(xk);
        dfdx = zeros(6,6);
        for j = 1:6
            dx = zeros(6,1); dx(j) = eps_jac;
            fxp = f_att(xk + dx);
            dfdx(:,j) = (fxp - fx0) / eps_jac;
        end
        A(:,:,k) = eye(6) + dt*dfdx;
    end

    % ----- Compute tracking error (optionally wrap yaw error) -----
    e = X - att_ref_horizon;
    % wrap yaw error to [-pi, pi] (helps if psi crosses ±pi)
    e(3,:) = atan2(sin(e(3,:)), cos(e(3,:)));

    % ----- Cost (scalar) -----
    J_stage = 0;
    for k = 1:Ha
        ek = e(:,k);
        uk = tau_guess(:,k);
        J_stage = J_stage + (ek'*Q*ek + uk'*R*uk);
    end
    J_stage = dt * J_stage;

    eN = e(:,Ha+1);
    J_terminal = eN' * Q * eN;   % using Q as terminal weight (OK for now)
    J_current = J_stage + J_terminal;

    % ----- Backward adjoint pass -----
    lambda = zeros(6, Ha+1);
    grad   = zeros(3, Ha);

    % terminal costate
    lambda(:,Ha+1) = 2 * Q * e(:,Ha+1);

    for k = Ha:-1:1
        % state cost gradient
        lambda(:,k) = 2*dt*Q*e(:,k) + A(:,:,k)' * lambda(:,k+1);

        % control gradient
        grad(:,k) = 2*dt*(R*tau_guess(:,k)) + dt*(g_att') * lambda(:,k+1);
    end

    % ----- Gradient descent update -----
    alpha_t = alpha / (1 + decay_rate * it);
    tau_guess = tau_guess - alpha_t * grad;

    % clamp (recommended while debugging)
    for k = 1:Ha
        tau_guess(:,k) = clamp_tau(tau_guess(:,k));
    end
end

end

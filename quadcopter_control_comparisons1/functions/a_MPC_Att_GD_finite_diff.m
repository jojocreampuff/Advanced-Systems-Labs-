function tau_guess = a_MPC_Att_GD_finite_diff(att_ref_horizon, X_current, tau_guess, Ha, Q, R, dt, Ix, Iy, Iz, att_iter, alpha, decay_rate, epsilon)
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

% Optional torque clamp (helps stability while debugging)
tau_max = [10;10;10]; % <-- adjust to your vehicle
clamp_tau = @(u) max(min(u, tau_max), -tau_max);

for it = 1:att_iter

    % ----- Forward rollout -----
    X = zeros(6, Ha+1);
    X(:,1) = X_current;

    for k = 1:Ha
        X(:,k+1) = X(:,k) + dt*(f_att(X(:,k)) + g_att*tau_guess(:,k));
    end

    e = X - att_ref_horizon;
    % e(3,:) = atan2(sin(e(3,:)), cos(e(3,:)));

    % ----- Cost (scalar) -----
    J_stage = 0;
    for k = 1:Ha
        J_stage = J_stage + (e(:,k)'*Q*e(:,k) + tau_guess(:,k)'*R*tau_guess(:,k));
    end
    J_stage = dt*J_stage;
    J_terminal = e(:,Ha+1)'*Q*e(:,Ha+1);
    J_current = J_stage + J_terminal;

    % ---- finite difference gradient ----
    grad = zeros(3,Ha);
    
    for k = 1:Ha
        for d = 1:3
            tau_pert = tau_guess;
            tau_pert(d,k) = tau_pert(d,k) + epsilon;
    
            % rollout perturbed
            Xp = zeros(6,Ha+1);
            Xp(:,1) = X_current;
    
            for j = 1:Ha
                Xp(:,j+1) = Xp(:,j) + dt*(f_att(Xp(:,j)) + g_att*tau_pert(:,j));
            end
    
            ep = Xp - att_ref_horizon;
            % ep(3,:) = atan2(sin(ep(3,:)), cos(ep(3,:)));
    
            Jp_stage = 0;
            for j = 1:Ha
                Jp_stage = Jp_stage + (ep(:,j)'*Q*ep(:,j) + tau_pert(:,j)'*R*tau_pert(:,j));
            end
            Jp_stage = dt*Jp_stage;
            Jp_terminal = ep(:,Ha+1)'*Q*ep(:,Ha+1);
            Jp = Jp_stage + Jp_terminal;
    
            grad(d,k) = (Jp - J_current) / epsilon;
        end
    end
    % ----- Gradient descent update -----
    alpha_t = alpha / (1 + decay_rate * it);
    tau_guess = tau_guess - alpha_t * grad;

    % % clamp (recommended while debugging)
    % for k = 1:Ha
    %     tau_guess(:,k) = clamp_tau(tau_guess(:,k));
    % end
end

end

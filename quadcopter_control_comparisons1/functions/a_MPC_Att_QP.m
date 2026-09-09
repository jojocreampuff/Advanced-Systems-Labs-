function [tau_guess] = a_MPC_Att_QP(pos_att_ref_horizon, X_current, tau_guess, Hp, Q, R, dt, Ix, Iy, Iz)
% QP-based nonlinear attitude MPC via SQP (sequential QP / RTI)
%
% State:  x = [phi theta psi p q r]'  (6x1)
% Input:  u = [tau_x tau_y tau_z]'    (3x1)
%
% pos_att_ref_horizon: 6 x (Hp+1) reference over horizon (preferred)
% X_current:           6 x 1 current attitude state
% tau_guess:           3 x Hp initial guess (warm start recommended)
% Q, R:                6x6 and 3x3 weights (diagonal ok)
% dt:                  timestep
% Ix,Iy,Iz:            inertias
%
% Output:
% tau_guess: optimized torque sequence (3 x Hp). Apply tau_guess(:,1).

Q = (1000*Q)/dt;
R = (10*R)/dt;

% ---------- user-tunable SQP settings ----------
SQP_iters = 3;      % 1-5 typical
eps_jac   = 1e-6;   % numerical jacobian step
T_horizon = Hp;     % already given as steps
% ---------------------------------------------

% Ensure ref horizon dimensions
if size(pos_att_ref_horizon,2) < Hp+1
    % if only Hp provided, hold last as terminal
    last = pos_att_ref_horizon(:,end);
    pos_att_ref_horizon(:,end+1:Hp+1) = repmat(last,1,Hp+1-size(pos_att_ref_horizon,2));
end

% Dynamics (your provided ones)
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

nx = 6; nu = 3;

% Flatten weights if diagonal for fast cost; otherwise use full matrices
Q_is_diag = isequal(Q, diag(diag(Q)));
R_is_diag = isequal(R, diag(diag(R)));
if Q_is_diag, Qd = diag(Q); end
if R_is_diag, Rd = diag(R); end

opts = optimoptions('quadprog','Display','off');

for it = 1:SQP_iters

    % ----- 1) Rollout nominal trajectory with current tau_guess -----
    Xnom = zeros(nx, Hp+1);
    Xnom(:,1) = X_current(:);
    for k = 1:Hp
        Xnom(:,k+1) = step(Xnom(:,k), tau_guess(:,k));
    end

    % ----- 2) Linearize dynamics along nominal trajectory -----
    % x_{k+1} ≈ A_k x_k + B_k u_k + c_k   (affine)
    Ak = zeros(nx,nx,Hp);
    Bk = repmat(dt*g_att,1,1,Hp); % since g_att constant and linear in u
    ck = zeros(nx,Hp);

    for k = 1:Hp
        xk = Xnom(:,k);
        uk = tau_guess(:,k);

        % Numerical Jacobian of step wrt x (A_k)
        % A_k = d/dx [ x + dt(f(x)+g u) ] = I + dt*df/dx
        J = zeros(nx,nx);
        fx0 = f_att(xk);
        for j = 1:nx
            dx = zeros(nx,1); dx(j) = eps_jac;
            fxp = f_att(xk + dx);
            J(:,j) = (fxp - fx0) / eps_jac;
        end
        Ak(:,:,k) = eye(nx) + dt*J;

        % Affine term so that linear model matches nominal exactly:
        % x_{k+1} = Ak xk + Bk uk + ck  -> ck = x_{k+1} - Ak xk - Bk uk
        xkp1 = Xnom(:,k+1);
        ck(:,k) = xkp1 - Ak(:,:,k)*xk - Bk(:,:,k)*uk;
    end

    % ----- 3) Build stacked prediction in deviations: dX = Sx*dx0 + Su*dU -----
    % Here dx0 = 0 because x0 is fixed; we solve for dU that improves tracking.
    % We predict deviations from nominal:
    % d x_{k+1} = Ak d x_k + Bk d u_k
    %
    % Stack dX = [dx1; ...; dxHp], dU = [du1; ...; duHp]

    Sx = zeros(nx*Hp, nx);
    Su = zeros(nx*Hp, nu*Hp);

    Phi = eye(nx);
    for i = 1:Hp
        % Phi_i = A_{i-1}...A_1
        if i == 1
            Phi = Ak(:,:,1);
        else
            Phi = Ak(:,:,i) * Phi;
        end
        Sx((i-1)*nx+1:i*nx, :) = Phi;

        % Fill Su blocks
        % dx_i depends on du_j for j<=i:
        % dx_i = A_{i-1}...A_{j+1} B_j du_j
        for j = 1:i
            % compute A_{i-1}...A_{j+1}
            M = eye(nx);
            for t = i-1:-1:j
                M = Ak(:,:,t+1) * M; % careful indexing
            end
            Bij = Bk(:,:,j);
            Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = M * Bij;
        end
    end

    % ----- 4) Stack nominal state targets and compute QP matrices -----
    % We want to minimize over horizon:
    % sum (x_nom + dx - x_ref)^T Q (x_nom + dx - x_ref) + (u_nom + du)^T R (u_nom + du)
    %
    % This becomes a QP in dU.

    % Stack nominal states x1..xHp
    Xnom_stack = reshape(Xnom(:,2:Hp+1), [], 1);
    Xref_stack = reshape(pos_att_ref_horizon(:,2:Hp+1), [], 1);
    d0 = (Xnom_stack - Xref_stack); % tracking error at nominal (stacked)

    % Block weights
    Qbar = dt * kron(eye(Hp), Q);  % dt scaling for stage
    Rbar = dt * kron(eye(Hp), R);

    % Nominal control stacked
    Unom_stack = reshape(tau_guess, [], 1);

    % Cost in dU:
    % J = (d0 + Su*dU)' Qbar (d0 + Su*dU) + (Unom + dU)' Rbar (Unom + dU) + const
    H = (Su' * Qbar * Su + Rbar);
    f = (Su' * Qbar * d0 + Rbar * Unom_stack);

    Hqp = 2*H;
    fqp = 2*f;

    % ----- 5) Solve QP for dU -----
    dU = quadprog(Hqp, fqp, [], [], [], [], [], [], [], opts);
    if isempty(dU)
        break;
    end

    % ----- 6) Update torque sequence -----
    Unew = Unom_stack + dU;
    tau_guess = reshape(Unew, nu, Hp);

end

end

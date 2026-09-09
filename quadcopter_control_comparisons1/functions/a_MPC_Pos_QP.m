function [pos_control_guess] = a_MPC_Pos_QP(pos_ref_horizon, X_current, pos_control_guess, Hp, Q, R, dt)
% QP-based linear MPC for position (double integrator)
% States:  x = [x y z u v w]'   (6x1)
% Inputs:  u = [ax ay az]'      (3x1)  (sign convention follows B_pos below)
%
% pos_ref_horizon is expected to be 6 x (Hp+1) (preferred) or 6 x Hp.
% This function returns the optimal control SEQUENCE (3 x Hp). Apply only the first column.
Q = 200*Q;
R = 1*R;

% --- Continuous-time linear model (double integrator) ---
A_pos = [0  0  0  1  0  0;
         0  0  0  0  1  0;
         0  0  0  0  0  1;
         0  0  0  0  0  0;
         0  0  0  0  0  0;
         0  0  0  0  0  0];

B_pos = [0   0   0;
         0   0   0;
         0   0   0;
        -1   0   0;
         0  -1   0;
         0   0  -1];

% --- Discretize with Euler (matches your simulation style) ---
A = eye(6) + dt*A_pos;
B = dt*B_pos;

% --- Build stacked reference Xref = [x1; x2; ...; xHp] ---
% Preferred: pos_ref_horizon is 6x(Hp+1) including x0_ref in col 1.
[nx, ncols] = size(pos_ref_horizon);

if ncols >= Hp+1
    % use reference for steps 1..Hp as cols 2..Hp+1
    Xref_stack = reshape(pos_ref_horizon(:,2:Hp+1), [], 1);  % (6Hp)x1
elseif ncols == Hp
    % assume provided reference already corresponds to steps 1..Hp
    Xref_stack = reshape(pos_ref_horizon(:,1:Hp), [], 1);    % (6Hp)x1
else
    error('pos_ref_horizon must have Hp or Hp+1 columns.');
end

% --- Build prediction matrices Sx, Su such that:
% X_stack = Sx*x0 + Su*U_stack
% where X_stack = [x1; x2; ...; xHp], U_stack = [u0; u1; ...; u_{Hp-1}]
nx = 6; nu = 3;
Sx = zeros(nx*Hp, nx);
Su = zeros(nx*Hp, nu*Hp);

% Precompute powers of A: A^0 .. A^Hp
Apow = cell(Hp+1,1);
Apow{1} = eye(nx);          % A^0
for k = 1:Hp
    Apow{k+1} = Apow{k}*A;  % A^k
end

for i = 1:Hp
    % block row for x_i (i steps ahead): A^i * x0
    Sx((i-1)*nx+1:i*nx, :) = Apow{i+1};  % A^i

    % contributions from controls u0..u_{i-1}
    for j = 1:i
        % x_i includes A^(i-j)*B*u_{j-1}
        Aij = Apow{i-j+1}; % A^(i-j)
        Su((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Aij * B;
    end
end

% --- Build block-diagonal weights (stage-only here; terminal handled by including Q at last state)
% We weight x1..xHp all with Q (common, simple). If you want terminal QN, replace last block.
% Qbar = kron(eye(Hp), Q);    % (6Hp)x(6Hp)
% Rbar = kron(eye(Hp), R);    % (3Hp)x(3Hp)
Qbar = dt * kron(eye(Hp), Q);
Rbar = dt * kron(eye(Hp), R);

% --- Form QP: minimize (X - Xref)'Qbar(X - Xref) + U'Rbar U
% Substitute X = Sx*x0 + Su*U:
% J(U) = U'(Su'QbarSu + Rbar)U + 2*(Sx*x0 - Xref)'QbarSu * U + const
x0 = X_current;
d  = (Sx*x0 - Xref_stack);  % (6Hp)x1

H = (Su' * Qbar * Su + Rbar);        % (3Hp)x(3Hp)
f = (Su' * Qbar * d);                % (3Hp)x1

% quadprog uses: 0.5*U'Hqp*U + fqp'*U
Hqp = 2*H;
fqp = 2*f;

% --- Optional: warm-start from provided guess ---
U0 = reshape(pos_control_guess(:,1:Hp), [], 1);  % (3Hp)x1

% --- Solve QP (unconstrained). Add bounds/constraints here if desired. ---
opts = optimoptions('quadprog', 'Display', 'off');
try
    % Some MATLAB versions support x0 as an extra argument; to stay compatible, call without it.
    Uopt = quadprog(Hqp, fqp, [], [], [], [], [], [], [], opts);

    % If solver returns empty (rare), fall back to the guess.
    if isempty(Uopt)
        Uopt = U0;
    end
catch
    % If Optimization Toolbox not available or solver fails, fall back to guess
    Uopt = U0;
end

% --- Return optimal control sequence as 3xHp ---
pos_control_guess = reshape(Uopt, nu, Hp);

end

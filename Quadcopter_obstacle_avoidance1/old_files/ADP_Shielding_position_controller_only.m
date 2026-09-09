%% CLF-CBF-QP Tracking for double integrator with input bounds
% Dynamics: xdot = A x + B u
% Error: e = x - xd(t)
% CLF: V = e' P e
% CBF (state constraint): ||e|| < 3 -> h(e) = 9 - e'e
% Barrier: Bbar = -log( h/(1+h) )  (logarithmic reciprocal barrier)
% Constraints:
%   CLF (soft): LfV + LgV*u + c*V - delta <= 0
%   CBF (hard): LfB + LgB*u - gamma/Bbar <= 0
%   Input: -1 <= u <= 1
%   (optional) delta >= 0
clear; clc; close all;
%% Define plant dynamics
f = @(x) [x(4); x(5); x(6); 0; 0; 0];
g = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
NS = 6;
NC = 3;
load("pos_controller_good_mae_low.mat", "W","R","Q")
W = W';

%% LQR conparison
A = [0  0   0   1   0   0   
        0   0   0   0   1   0   
        0   0   0   0   0   1   
        0   0   0   0   0   0  
        0   0   0   0   0   0   
        0   0   0   0   0   0];

B = [0  0   0  
        0   0   0
        0   0   0
        1   0   0
        0   1   0
        0   0   1];

[K, P, z]= lqr(A,B,Q,R);

%% Simulation settings
dt = 1e-2;
Tf = 50;
N  = round(Tf/dt);

T = (0:N)*dt;

x = zeros(NS, N+1);
u = zeros(NC, N+1);
ref_save = zeros(6,N);
delta = zeros(1, N+1);

ref = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);  % reference_x
             (1-exp(-0.01*t))*9.81*sin(0.2*t);  % reference_y
             .1*t;
             (sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000 ;
                (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5;
                                                                            1/10];                  
ref_dot = @(t) [(sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000 ;
                (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5;
                                                                            1/10;
(cos(t/5)*((981*exp(-t/100))/100 - 981/100))/25 - (981*cos(t/5)*exp(-t/100))/1000000 - (981*sin(t/5)*exp(-t/100))/25000
(sin(t/5)*((981*exp(-t/100))/100 - 981/100))/25 + (981*cos(t/5)*exp(-t/100))/25000 - (981*sin(t/5)*exp(-t/100))/1000000
                                                                              0];
%% barrier function
radius = 1.5;
c = [-1;-1.5;2];

h_x = @(x,c,r) (x(1)-c(1)).^2 + (x(2)-c(2)).^2 + (x(3)-c(3)).^2 - r.^2; % > 0

delB_delx = @(x,c,r) (2*x(1)-2*c(1)) / (h_x(x,c,r)*(1+h_x(x,c,r))); 
delB_dely = @(x,c,r) (2*x(2)-2*c(2)) / (h_x(x,c,r)*(1+h_x(x,c,r)));
delB_delz = @(x,c,r) (2*x(3)-2*c(3)) / (h_x(x,c,r)*(1+h_x(x,c,r)));
gradB = @(x,c,r) [delB_delx(x,c,r); delB_dely(x,c,r); delB_delz(x,c,r); 0; 0; 0];

for i = 1:N
    t = T(i);
    ref_save(:,i) = ref(t);
end

figure; hold on; grid on; axis equal;
plot3(ref_save(1,:), ref_save(2,:), ref_save(3,:), 'LineWidth', 1.5);
nSphere = 30;                              % resolution
[Xs,Ys,Zs] = sphere(nSphere);              % unit sphere
Xs = radius*Xs + c(1);
Ys = radius*Ys + c(2);
Zs = radius*Zs + c(3);

surf(Xs, Ys, Zs,'FaceAlpha', 0.20,'EdgeAlpha', 0.10,'FaceColor', [1 0 0],'EdgeColor', 'k');

plot3(c(1), c(2), c(3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

xlabel("x (m)"); ylabel("y (m)"); zlabel("z (m)");
title('Trajectory with spherical unsafe zone (h_x<0 inside)');

%% CLF-CBF-QP parameters
% stabilizing terms 
c_clf     = 1;     % CLF rate 
p_delta   = 1;     % penalty on delta^2 (big => stability over saftey)
% safety terms 
gamma_cbf = 1.0;     % CBF gamma in  gamma/Bbar
k0 = 20;                   % stiffness
k1 = 2*sqrt(k0);           % damping

u_min = -10;
u_max =  10;

% initial condition (pick something safe)
x(:,1) = 1*randn(NS,1);  % you can change

% quadprog options
opts = optimoptions('quadprog', 'Display','off');

%% Main loop
for k = 1:N
    t = T(k);
    ref_save(:,k) = ref(t);
    % current error
    e_k = x(:,k) - ref(t);
    e(:,k) = e_k;

    % ---- Step 1: nominal LQR input (unsafe nominal)
    % Your requested nominal: u_nom = -K*e
    u_nom = -0.5*R^-1*g'*grad_Basis_func_pos(e_k)'*W;

    % ---- Error drift term d(t) used in Lf(.) calculations
    % d = A*xd(t) - xd_dot(t);   % = [0; sin(t)] this is infact the feed 
    % forward term to transform xdot = Ax + Bu to edot = Ae + Bu
    % +feedforward terms
    f_e = A*e_k + A*ref(t) - ref_dot(t);           % drift in error dynamics (without control)

    % ---- Step 2: CLF V(e)=e'Pe and soft constraint
    f_x = f(x(:,k));
    V = W'*Basis_Func_pos(x(:,k));                 % CLF
    gradV = grad_Basis_func_pos(e_k)'*W;                % dV/de
    LfV = gradV' * (f_x);                 % along f_e
    LgV = gradV' * g;                   % 1x3

    % CLF inequality: [LgV, -1]*[u;delta] <= -LfV - c*V
    A_clf = [LgV, -1];
    b_clf = -LfV - c_clf*V;

    % ---- Step 3: h(x) and cbf
    h = h_x(x(:,k),c,radius);
    htraj(k) = h;

    % Guard against infeasible/undefined barrier if already unsafe
    eps_h = 1e-10;
    if h <= eps_h
        % If you start outside, the barrier is undefined; clip for numerics.
        % In real use, you should ensure initial condition satisfies h>0.
        h = eps_h;
    end

    % Barrier function: Bbar = -log(h/(1+h))
    Bbar = -log( h/(1+h) );
    % second order (relative degree 2 CBF) LgB = 0 for this B(x), need barrier to include controls
    % Lf_Lf_h + Lg_Lf_h*u + k1*Lf_h + k0*h + gamma_cbf/Bbar >= 0 (zeroing barrier function)
    % -Lg_Lf_h*u <= Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar
    % [-Lg_Lf_h, 0]* [u ; delta] <= Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar
    %          A_cbf * U        <=  B_cbf
    position = x(1:3,k);
    velocity = x(4:6,k);
    % h = (position - c)'*(position - c) - r^2
    Lf_h = 2*(position - c)'*velocity;          % grad_h' * f(x)
    Lf_Lf_h = 2*(velocity'*velocity);           % grad(grad_h' * f(x))' * f(x)
    Lg_Lf_h = 2*(position - c)';                % grad(grad_h' * f(x))' * g(x)

    A_cbf = [-Lg_Lf_h, 0]; % 1x4
    b_cbf = Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar; % 1x1

    % ---- Step 4: CLF-CBF QP
    % Decision U = [u; delta]
    %
    % Objective: minimize (u - u_nom)^2 + p_delta * delta^2
    % In standard quadprog form: 0.5*U'HU + f'U
    H   = blkdiag(2*eye(NC), 2*p_delta);    % 4x4
    fqp = [-2*u_nom; 0];                   % 4x1

    % Additional constraints:
    %  -1 <= u <= 1
    A_u = [ eye(NC),  zeros(NC,1);
           -eye(NC),  zeros(NC,1) ];     % 6x4
    b_u = [ u_max*ones(NC,1);
           -u_min*ones(NC,1) ];          % 6x1

    % delta >= 0 (optional but standard for "softening")
    A_del = [zeros(1,NC), -1];   % 1x4
    b_del = 0;

    % Stack constraints: A*U <= b
    Aqp = [A_clf;
           A_cbf;
           A_u;
           A_del];

    bqp = [b_clf;
           b_cbf;
           b_u;
           b_del];

    % Solve QP
    [Uopt,~,exitflag] = quadprog(H, fqp, Aqp, bqp, [], [], [], [], [], opts);

    if exitflag <= 0 || isempty(Uopt)
        % Fallback: saturate nominal if QP fails (should be rare if feasible)
        u_k = min(max(u_nom, u_min), u_max);
        del_k = 0;
    else
        u_k = Uopt(1:3);
        del_k = Uopt(4);
    end

    u(:,k) = u_k;
    delta(k) = del_k;

    % Integrate actual system (Euler)
    xdot = A*x(:,k) + B*u_k;
    x(:,k+1) = x(:,k) + dt*xdot;
end


%% Plots
figure; 
subplot(3,1,1);
plot(T, x(1,:), 'LineWidth', 1.2); hold on;
plot(T(1:end-1), ref_save(1,:), '--', 'LineWidth', 1.2); % (will error because xd expects scalar)
legend('x_1','xd_1'); title('x_1 vs desired'); grid on;

subplot(3,1,2);
plot(T, x(2,:), 'LineWidth', 1.2); hold on;
plot(T(1:end-1), ref_save(2,:), 'LineWidth', 1.2);
legend('x_2','xd_2'); title('x2 vs desired'); grid on;

subplot(3,1,3);
plot(T, x(3,:), 'LineWidth', 1.2); hold on;
plot(T(1:end-1), ref_save(3,:), 'LineWidth', 1.2);
legend('x_3','xd_3'); title('x3 vs desired'); grid on;

figure;
subplot(3,1,1);
plot(T, u(1,:), 'LineWidth', 1.2); hold on;
yline(u_max,'--'); yline(u_min,'--');
legend('u1','u_{max}','u_{min}'); title('Control input (bounded)'); grid on;
subplot(3,1,2);
plot(T, u(2,:), 'LineWidth', 1.2); hold on;
yline(u_max,'--'); yline(u_min,'--');
legend('u2','u_{max}','u_{min}'); grid on;
subplot(3,1,3);
plot(T, u(3,:), 'LineWidth', 1.2); hold on;
yline(u_max,'--'); yline(u_min,'--');
legend('u3','u_{max}','u_{min}'); grid on;

% figure;
% plot(T, delta, 'LineWidth', 1.2);
% title('\delta (CLF relaxation)'); grid on;

figure;
plot(T(1:end-1), htraj, 'LineWidth', 1.2); yline(0,'--');
title('h(x) (must stay > 0)'); grid on;
legend("h(x)")

figure; hold on; grid on;
plot3(x(1,:),x(2,:), x(3,:),Color=[0,0,1],LineStyle="-")
plot3(ref_save(1,:), ref_save(2,:), ref_save(3,:),Color=[0,1,0],LineStyle="--")
surf(Xs, Ys, Zs, ...
    'FaceAlpha', 0.10, ...                 % translucent
    'EdgeAlpha', 0.10, ...                 % faint mesh
    'FaceColor', [1 0 0], ...              % red unsafe boundary
    'EdgeColor', 'k');
xlabel("x (m)"); ylabel("y (m)"); zlabel("z (m)");
legend("safe trajectory", "reference","unsafe region")


%% CLF-CBF-QP Tracking for double integrator with input bounds
% Barrier: Bbar = -log( h/(1+h) )  (logarithmic reciprocal barrier)
% Constraints:
%% CLF (soft): LfV + LgV*u + c*V - delta <= 0
    % DQ: format [LgV, -1]*[u;delta] <= -LfV - c*V
    %            A_clf * U  < = B_clf
%% CBF (hard): LfB + LgB*u - gamma/Bbar <= 0 <--- first order barrier method
% second order (relative degree 2 CBF) LgB = 0 for this B(x), need barrier to include controls
        % Lf_Lf_h + Lg_Lf_h*u + k1*Lf_h + k0*h + gamma_cbf/Bbar >= 0 (zeroing barrier function)
        % -Lg_Lf_h*u <= Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar
        % [-Lg_Lf_h, 0]* [u ; delta] <= Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar
        %          A_cbf * U        <=  B_cbf

%% QP format: 
        % Must look like this Ax<=b
        % where x is what is to be minimized, in our case U = [u; delta]
%% additonal QP constraints 
%   Input: -1 <= u <= 1
%   (optional) delta >= 0
clear; clc; close all;

%% Define position control plant dynamics
% making g positive
g = 9.81;
f_pos = @(x) [x(4); x(5); x(6); 0; 0; g];
g_pos = [0 0 0; 0 0 0; 0 0 0; -1 0 0; 0 -1 0; 0 0 -1];
N_pos_states = 6;
N_pos_control = 3;
load("pos_controller_good_mae_low.mat", "W","R","Q")
W_pos = W';
R_pos = R;
Q_pos = Q;
load("holy_crap_it_worked3.mat", "W", "R", "Q")
W_att = W';
R_att = R;
Q_att = Q;
%% Simulation settings
dt = 0.001;
Tf = 50;
N  = round(Tf/dt);
T = (0:N)*dt;
tf = 50;
Ix = 0.3;   % Moment of inertia (kg*m^2)
Iy = 0.4;
Iz = 0.5;
m = 1;
X = zeros(12,length(T));
U = zeros(4, length(T));  % Control input trajectory
X_ref = zeros(12, length(T));  % Reference trajectory
angles = zeros(3,length(T));
r_yaw = pi/4*ones(1,N);

x_pos = zeros(N_pos_states, N+1);
ref_save = zeros(6,N);
delta = zeros(1, N+1);
U_pos = zeros(N_pos_control, N+1);
U_pos_filt = U_pos;
% cost plotting 
instant_cost_pos = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_pos = zeros(1, length(T)); % Integrated cost
instant_cost_att = zeros(1, length(T)); % Instantaneous cost
cumulative_cost_att = zeros(1, length(T)); % Integrated cost

%% Position Reference
ref = @(t) [(1-exp(-0.01*t))*9.81*cos(0.2*t);  % reference_x
             (1-exp(-0.01*t))*9.81*sin(0.2*t);  % reference_y
             -.1*t;
             (sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000 ;
                (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5;
                                                                            -1/10];                  
ref_dot = @(t) [(sin(t/5)*((981*exp(-t/100))/100 - 981/100))/5 + (981*cos(t/5)*exp(-t/100))/10000 ;
                (981*sin(t/5)*exp(-t/100))/10000 - (cos(t/5)*((981*exp(-t/100))/100 - 981/100))/5;
                                                                            -1/10;
(cos(t/5)*((981*exp(-t/100))/100 - 981/100))/25 - (981*cos(t/5)*exp(-t/100))/1000000 - (981*sin(t/5)*exp(-t/100))/25000
(sin(t/5)*((981*exp(-t/100))/100 - 981/100))/25 + (981*cos(t/5)*exp(-t/100))/25000 - (981*sin(t/5)*exp(-t/100))/1000000
                                                                              0];
for j = 1:length(T)
    t = T(j);
    ref_save(:,j) = ref(t);
end

%% barrier function: a 3D sphere
radius = 2;
c = [-1;-1.5;-2];

% h(x) is the constraint B(x) is the barrier
h_x = @(x,c,r) (x(1)-c(1)).^2 + (x(2)-c(2)).^2 + (x(3)-c(3)).^2 - r.^2; % > 0
B_x = @(h) -log( h/(1+h) );
% delB_delx = @(x,c,r) (2*x(1)-2*c(1)) / (h_x(x,c,r)*(1+h_x(x,c,r))); 
% delB_dely = @(x,c,r) (2*x(2)-2*c(2)) / (h_x(x,c,r)*(1+h_x(x,c,r)));
% delB_delz = @(x,c,r) (2*x(3)-2*c(3)) / (h_x(x,c,r)*(1+h_x(x,c,r)));
% gradB = @(x,c,r) [delB_delx(x,c,r); delB_dely(x,c,r); delB_delz(x,c,r); 0; 0; 0];

%% position control CLF-CBF-QP parameters
% stabilizing terms 
c_clf     = 1;     % CLF rate 
p_delta   = 1;     % penalty on delta^2 (big => stability over saftey)
% safety terms 
gamma_cbf = .05;     % CBF gamma in  gamma/Bbar
k0 = 1;                   % stiffness
k1 = 2*sqrt(k0);           % damping

% max and min control values
u_min = -20;
u_max =  20;

% quadprog options
opts = optimoptions('quadprog', 'Display','off');
X(:,1) = [5; 5; 0; zeros(9,1)];
pqr_filt = zeros(3,1);

tic

%% Main loop
for i = 1:N
    t = T(i);
    x_pos = X(1:6,i);
    X_ref(1:6,i) = ref(t);
    e_i = x_pos - X_ref(1:6,i);
    pos_error(:,i) = e_i;
    %% Position control
    % ---- Step 1: nominal LQR input (unsafe nominal) (nominal control must include gravity)
    u_nom = -0.5*R_pos^-1*g_pos'*grad_Basis_func_pos(e_i)'*W_pos + [0; 0; g];
    f_e = f_pos(x_pos) - ref_dot(t);           % drift in error dynamics (without control)
    % ---- Step 2: CLF V(e)=e'Pe and soft constraint
    f_x = f_pos(x_pos);
    V = W_pos'*Basis_Func_pos(x_pos);           % approxminated Value function needed for CLF
    gradV = grad_Basis_func_pos(e_i)'*W_pos;    % approximated dV/de needed for CLF
    LfV = gradV' * (f_e);                       % along f_e
    LgV = gradV' * g_pos;                   % 1x3
    % CLF inequality: [LgV, -1]*[u;delta] <= -LfV - c*V
    A_clf = [LgV, -1];
    b_clf = -LfV - c_clf*V;

    % ---- Step 3: h(x) and cbf
    h = h_x(x_pos,c,radius);
    htraj(i) = h;
    eps_h = 1e-10;
    if h <= eps_h
        h = eps_h;
    end

    Bbar = B_x(h);
    position = x_pos(1:3);
    velocity = x_pos(4:6);
    Lf_h = 2*(position - c)'*velocity;          % grad_h' * f(x)
    Lf_Lf_h = 2*(velocity'*velocity) + 2*(position - c)'*[0;0;g];           % grad(grad_h' * f(x))' * f(x)
    Lg_Lf_h = -2*(position - c)';                % grad(grad_h' * f(x))' * g(x)

    A_cbf = [-Lg_Lf_h, 0]; % 1x4
    b_cbf = Lf_Lf_h + k1*Lf_h + k0*h + gamma_cbf/Bbar; % 1x1

    % ---- Step 4: CLF-CBF QP
    % Decision U = [u; delta]
    % Objective: minimize (u - u_nom)^2 + p_delta * delta^2
    % In standard quadprog form: 0.5*U'HU + f'U
    H   = blkdiag(2*eye(N_pos_control), 2*p_delta);    % 4x4
    fqp = [-2*u_nom; 0];                   % 4x1

    % Additional constraints:
    %  -1 <= u <= 1
    A_u = [ eye(N_pos_control),  zeros(N_pos_control,1);
           -eye(N_pos_control),  zeros(N_pos_control,1) ];     % 6x4
    b_u = [ u_max*ones(N_pos_control,1);
           -u_min*ones(N_pos_control,1) ];          % 6x1

    % delta >= 0 (optional but standard for "softening")
    A_del = [zeros(1,N_pos_control), -1];   % 1x4
    b_del = 0;

    % Stack constraints: A*U <= b
    Aqp = [A_clf;A_cbf; A_u; A_del];
    bqp = [b_clf; b_cbf; b_u; b_del];

    % Solve QP only if getting unsafe
    if h < 3
        [Uopt,~,exitflag] = quadprog(H, fqp, Aqp, bqp, [], [], [], [], [], opts);
        if exitflag <= 0 || isempty(Uopt)
            U_pos(:,i) = min(max(u_nom, u_min), u_max);
            delta(i) = 0;
        else
            U_pos(:,i) = Uopt(1:3);
            delta(i) = Uopt(4);
        end

    else
        U_pos(:,i) = u_nom;
    end

    [ft, r_pitch, r_roll] = sys_solve(-U_pos(1,i), -U_pos(2,i), -U_pos(3,i), r_yaw(i), m);

    %% att control (no safety enforced)
    angles(:,i) = [r_pitch; r_roll; r_yaw(i)];
    % [pqr, pqr_filt] = deriv_filt(angles,i,dt,pqr_filt);
    pqr = deriv(angles,i,dt);
    X_ref(7:12, i) = [angles(:,i); pqr];
    Att_error(:,i) = X(7:12,i) - X_ref(7:12, i);
    torques = -0.5*R_att^-1*att_G(X(7:12,i))'*grad_Basis_func_att(Att_error(:,i))'*W_att;

    U(:,i) = [ft; torques];

    % State update using Euler integration
    if i < length(T)
        X(:, i+1) = X(:, i) + dt * (Full_f_225(X(:, i),g,Ix,Iy,Iz) + Full_g_225(X(:, i), m,Ix,Iy,Iz)* U(:, i));
    end
    instant_cost_pos(i) = pos_error(:,i)' * Q_pos * pos_error(:,i) + U_pos(:,i)' * R_pos * U_pos(:,i);
    instant_cost_att(i) = Att_error(:,i)' * Q_att * Att_error(:,i) + torques' * R_att * torques;
   if i > 1
        cumulative_cost_pos(i) = cumulative_cost_pos(i-1) + dt*instant_cost_pos(i);
        cumulative_cost_att(i) = cumulative_cost_att(i-1) + dt*instant_cost_att(i);
   end

end
DSC_final_time = toc;
DSC_results.X = X;
DSC_results.U = U;
DCS_results.h_log = htraj;
DSC_results.error = [pos_error;Att_error];
DSC_results.r_initial = ref_save;
DSC_results.inscost = [instant_cost_pos;instant_cost_att];
DSC_results.cumcost = [cumulative_cost_pos;cumulative_cost_att];
DSC_results.simtime = DSC_final_time;
DSC_error = DSC_results.error;
DSC_tracking_error = (DSC_error(1,:).^2 + DSC_error(2,:).^2 + DSC_error(3,:).^2).^0.5;
DSC_tracking_error_sum = dt*sum(DSC_tracking_error,2)
DSC_c_cost_total = sum(DSC_results.cumcost, 1);
cost = DSC_c_cost_total(end-1);

% save("single_critic_r2.mat","DSC_tracking_error","time")

% figure;
% titles = {'x', 'y', 'z', 'u', 'v', 'w', '$\phi$', '$\theta$', '$\psi$', 'p', 'q', 'r'};
% for i = 1:12
%     subplot(4,3,i);
%     plot(T, X(i,:), 'b', 'LineWidth', 1.5); hold on;
%     plot(T, X_ref(i,:), 'r--', 'LineWidth', 1.5);
%     xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i}, 'Interpreter', 'latex');
% 
%     grid on;
% end
% legend('O-DSC', 'Reference');
% sgtitle('LQR X DSC Tracking: All states');

figure;
grid on
hold on
plot3(ref_save(1,:), ref_save(2,:), -ref_save(3,:), '--', 'Color', [0 1 0], 'Linewidth', 1.5)
plot3(X(1,:), X(2,:), -X(3,:), 'b', 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:10000:length(X(1,:)), 'MarkerSize', 5)
nSphere = 30;                              % resolution
[Xs,Ys,Zs] = sphere(nSphere);              % unit sphere
Xs = radius*Xs + c(1);
Ys = radius*Ys + c(2);
Zs = radius*Zs + c(3);
surf(Xs, Ys, -Zs,'FaceAlpha', 0.20,'EdgeAlpha', 0.10,'FaceColor', [1 0 0],'EdgeColor', 'k');
xlabel('$x$ (m)', 'Interpreter', 'latex')
ylabel('$y$ (m)', 'Interpreter', 'latex')
zlabel('$z$ (m)', 'Interpreter', 'latex')
legend(["$r(t)$", "O-SNAC", "$h(x) = 0$"], 'Interpreter', 'latex');

% figure; hold on; grid on;
% plot3(X(1,:),X(2,:), -X(3,:),Color=[0,0,1],LineStyle="-")
% plot3(ref_save(1,:), ref_save(2,:), -ref_save(3,:),Color=[0,1,0],LineStyle="--")
% surf(Xs, Ys, -Zs,'FaceAlpha', 0.20,'EdgeAlpha', 0.10,'FaceColor', [1 0 0],'EdgeColor', 'k');
% xlabel("x (m)"); ylabel("y (m)"); zlabel("z (m)");
% legend("safe trajectory", "reference","unsafe region")

%% Plots
% figure; 
% subplot(3,1,1);
% plot(T, X(1,:), 'LineWidth', 1.2); hold on;
% plot(T, ref_save(1,:), '--', 'LineWidth', 1.2); % (will error because xd expects scalar)
% legend('x_1','xd_1'); title('x_1 vs desired'); grid on;
% subplot(3,1,2);
% plot(T, X(2,:), 'LineWidth', 1.2); hold on;
% plot(T, ref_save(2,:), 'LineWidth', 1.2);
% legend('x_2','xd_2'); title('x2 vs desired'); grid on;
% subplot(3,1,3);
% plot(T, -X(3,:), 'LineWidth', 1.2); hold on;
% plot(T, -ref_save(3,:), 'LineWidth', 1.2);
% legend('x_3','xd_3'); title('x3 vs desired'); grid on;

% figure;
% subplot(3,1,1);
% plot(T, U_pos(1,:), 'LineWidth', 1.2); hold on;
% yline(u_max,'--'); yline(u_min,'--');
% legend('u1','u_{max}','u_{min}'); title('Control input (bounded)'); grid on;
% subplot(3,1,2);
% plot(T, U_pos(2,:), 'LineWidth', 1.2); hold on;
% yline(u_max,'--'); yline(u_min,'--');
% legend('u2','u_{max}','u_{min}'); grid on;
% subplot(3,1,3);
% plot(T, -U_pos(3,:), 'LineWidth', 1.2); hold on;
% yline(u_max,'--'); yline(u_min,'--');
% legend('u3','u_{max}','u_{min}'); grid on;

figure;
plot(T(1:end-1), htraj, 'LineWidth', 1.2);
yline(0,'r-');
xlabel("Time (s)", 'Interpreter','latex');
ylabel("$h(x)$",   'Interpreter','latex');
grid on;
legend("h(x)", 'Interpreter','latex')


% figure;
% titles = {'x', 'y', 'z', 'u', 'v', 'w', '$\phi$', '$\theta$', '$\psi$', 'p', 'q', 'r'};
% for i = 1:12
%     subplot(4,3,i);
%     plot(T, X(i,:), 'b', 'LineWidth', 1.5); hold on;
%     plot(T, X_ref(i,:), 'r--', 'LineWidth', 1.5);
%     xlabel('Time (s)', 'Interpreter', 'latex'); ylabel(titles{i}, 'Interpreter', 'latex');
% 
%     grid on;
% end
% legend('O-DSC', 'Reference');
% sgtitle('DSC Tracking: All states');


function [ft, pitch, roll] = sys_solve(u1, u2, u3, psi, m) % u1 = ux , u2 = uy , u3 = uz - g  
    a = sin(psi);
    b = cos(psi);
    A = u1 / (u3);
    B = u2 / (u3);
    C = (a*A - b*B) / ( (a^2) + (b^2) );
    roll = atan( (B + b*C) / a );
    pitch = atan( C * cos(roll) );
    % saftey limit threshold on desired pitch and roll
    max_angle = pi/6;
    if pitch>max_angle
        pitch = max_angle;
    elseif pitch<-max_angle
        pitch = -max_angle;
    end
    if roll>max_angle
        roll = max_angle;
    elseif roll<-max_angle
        roll = -max_angle;
    end
    ft = ( m / (cos(pitch)*cos(roll)) ) * (-u3); 

end

function [pqr, pqr_filt] = deriv_filt(angles, i, dt, pqr_filt)
    alpha = 0.8;
    pqr_max = 5;
if i == 1
    pqr_raw = ((angles(:, i) - 0)/dt);
    pqr = alpha * pqr_filt + (1-alpha)* pqr_raw;
elseif i > 1
    pqr_raw = ((angles(:, i) - angles(:, i - 1))/(dt));
    pqr = alpha * pqr_filt + (1-alpha)* pqr_raw;
end
pqr_filt = pqr;
% Limit pqr to be between -5 and 5
pqr(pqr > pqr_max) = pqr_max;
pqr(pqr < -pqr_max) = -pqr_max;

% omega_d_dot_raw = angle_rate_solver(ohm_d,i,dt); % assumes this returns derivative of a vector input
% alpha = 0.9;
% omega_d_dot = alpha * omega_d_dot_filt + (1-alpha) * omega_d_dot_raw;
% omega_d_dot_filt = omega_d_dot;
end

function pqr = deriv(angles, i, dt)
    pqr_max = 5;
if i == 1
    pqr = ((angles(:, i) - 0)/dt);
    pqr(pqr > 1) = 1;
    pqr(pqr < -1) = -1;
elseif i > 1
    pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
    % Limit pqr to be between -5 and 5
    pqr(pqr > pqr_max) = pqr_max;
    pqr(pqr < -pqr_max) = -pqr_max;
end


end
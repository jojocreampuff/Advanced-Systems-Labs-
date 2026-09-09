clear; clc; close all;
%% Online SNAC tracking
global Q R alpha N_states N_neurons tf amp_mult N_R theta_star eps_reg rr c gamma u_max cb
tf = 900;
R = .1;
Q = diag([500,1]);
alpha = 10;
N_states = 2;
N_R = N_states;
N_neurons = length(PHI(ones(N_states,1), ones(N_states,1)));
amp_mult = 1;

rr = 1.5;
c = [0;0];
gamma = 1e-2;
eps_reg = 1e-6;
u_max = 20;
cb = 0.0002;

A = [0 1; -1 1];
B = [0; 1];

[P,K,~] = icare(A,B,Q,R);
W_star = zeros(N_neurons, N_states+N_R); 
W_star(1,1:2) = [P(1,1), P(1,2)];
W_star(2,1:2) = [P(2,1), P(2,2)];
theta_star = reshape(W_star,[],1);
W_init = zeros(N_neurons, N_states+N_R);
W_init(1,1:2) = [1, 1];
W_init(2,1:2) = [1, 1];

% train both safe and unsafe tracking problem
theta_init = reshape(W_init,[],1);
X0 = zeros(N_states,1);
initials = [X0;theta_init];

%% (1) train unsafe network (for comparision)
fprintf("Training UNSAFE (barrier OFF)...\n");
[t1,x1]= ode23(@online_snac_training_tracking, [0 tf], initials); %% CHATGPT: this is a functional training function for the optimal SNAC tracking problem. 
% extract states
state_history1 = x1(:, 1:N_states);
error_history1 = 0*state_history1;
for j = 1:length(t1)
    [ref_training, ~] = ref_refdot(t1(j));
    error_history1(j,:) = state_history1(j,:) - ref_training';
end
W_history1 = x1(:,N_states+1:end);
W_unsafe = W_history1(end,:);
W_unsafe = reshape(W_unsafe, N_neurons,N_states+N_R);

figure;
subplot(2,1,1)
plot(t1,error_history1(:,1:N_states));
title ('Errors During training w/o safety');
xlabel ('Time (s)');
subplot(2,1,2)
plot(t1,W_history1);
title ('Parameters of the NN');
xlabel ('Time (s)');

%% (2) Train safe network (with b(x) inside cost and ub)
fprintf("Training SAFE (barrier ON)...\n");
% ode_opts = odeset('RelTol',1e-3,'AbsTol',1e-6,'MaxStep',0.1); % default
ode_opts = odeset('RelTol',1e-6,'AbsTol',1e-7,'MaxStep',0.1); 
[t2,x2]= ode23(@online_snac_training_SAFE_tracking, [0 tf], initials, ode_opts);

% extract states
state_history2 = x2(:, 1:N_states);
error_history2 = 0*state_history2;
for j = 1:length(t2)
    [ref_training, ~] = ref_refdot(t2(j));
    error_history2(j,:) = state_history2(j,:) - ref_training';
end
W_history2 = x2(:,N_states+1:end);
W_safe = W_history2(end,:);
W_safe = reshape(W_safe, N_neurons,N_states+N_R);

figure;
subplot(2,1,1)
plot(t2,error_history2(:,1:N_states));
title ('Errors during training with safety');
xlabel ('Time (s)');
subplot(2,1,2)
plot(t2,W_history2);
title ('Parameters of the NN');
xlabel ('Time (s)');

%% simulation
T = 10;
dt = 0.01;
N = T/dt;
time = dt:dt:T;
x_online_safe = zeros(N_states,N);
x_online_safe(:,1) = [1;1];
u_online_safe = zeros(1,N);
x_online_unsafe = zeros(N_states,N);
x_online_unsafe(:,1) = x_online_safe(:,1);
u_online_unsafe = zeros(1,N);
reference = zeros(2,N);

f = @(x) [x(2);
    (1-x(1).^2).*x(2) - x(1)];
g = [ 0 ;  1 ];
G_z = [g; 0*g];

params.barrier.c = c;
params.barrier.r = rr;
params.barrier.gamma = gamma;
T = 0;
for i = 1:N
    T = dt*i;
    [reference(:,i), ref_dot] = ref_refdot(T);

    %% safe simulation
    phi = PHI(x_online_safe(:,i) - reference(:,i),  reference(:,i));
    [~,gradB,h] = B_x(x_online_safe(:,i), params);
    h_log(i) = h;

    u_online_safe(:,i) = (-0.5*R^-1 * G_z' * W_safe'*phi) -cb*R^-1 * g' * gradB;
    u_online_safe(:,i) = max(min(u_online_safe(:,i), u_max), -u_max);
    x_online_safe(:, i+1) = x_online_safe(:,i) + dt *( f(x_online_safe(:,i)) + g* u_online_safe(:,i));

    %% unsafe simulation
    phi = PHI(x_online_unsafe(:,i) - reference(:,i),  reference(:,i));
    u_online_unsafe(:,i) = -0.5*R^-1 * G_z' * W_unsafe'*phi;
    u_online_unsafe(:,i) = max(min(u_online_unsafe(:,i), u_max), -u_max);
    [~,gradB,h] = B_x(x_online_unsafe(:,i), params);
    h_log_unsafe(i) = h;
    x_online_unsafe(:, i+1) = x_online_unsafe(:,i) + dt *( f(x_online_unsafe(:,i)) + g* u_online_unsafe(:,i));
end

% figure;
% subplot(3,1,1)
% hold on
% plot(time, x_online(1,1:end-1), '--r *', 'LineWidth', 0.9, 'MarkerIndices',1:100:length(time), "MarkerSize",5)
% grid on;
% ax = gca; ax.Box = 'on'; ax.LineWidth = .5; ax.XColor = 'k'; ax.YColor = 'k';
% plot(time, reference(1,:),"Color","b","LineStyle","-","LineWidth",1)
% xlabel("Time (s)", 'Interpreter', 'latex')
% ylabel("$x_{1}$", 'Interpreter', 'latex')
% legend("Online SNAC", "$r$", 'Interpreter', 'latex')
% 
% subplot(3,1,2)
% hold on
% plot(time, x_online(2,1:end-1), '--r *', 'LineWidth', 0.9, 'MarkerIndices',1:100:length(time), "MarkerSize",5)
% plot(time, reference(2,:),"Color","b","LineStyle","-","LineWidth",1)
% xlabel("Time (s)", 'Interpreter', 'latex')
% ylabel("$x_{2}$", 'Interpreter', 'latex')
% legend("Online SNAC", "$\dot{r}$", 'Interpreter', 'latex')
% grid on;
% ax = gca; ax.Box = 'on'; ax.LineWidth = .5; ax.XColor = 'k'; ax.YColor = 'k';
% title("$x_{2}$ Trajectory Tracking", 'Interpreter', 'latex')

figure;
hold on;
plot(time, h_log)
plot(time, h_log_unsafe)
xlabel("Time (s)", 'Interpreter', 'latex')
ylabel("$h(x)$", 'Interpreter', 'latex')
grid on;
ax = gca; ax.Box = 'on'; ax.LineWidth = .5; ax.XColor = 'k'; ax.YColor = 'k';
legend("Safe","Unsafe")
title("Boundry plot", 'Interpreter', 'latex')

theta = linspace(0, 2*pi, 100);
bound_x1 = c(1) + rr * cos(theta);
bound_x2 = c(2) + rr * sin(theta);

figure;
hold on;
grid on;
plot(x_online_safe(1,:),x_online_safe(2,:), '-.', ...
    'Color', [1 0 0], 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:500:length(time), 'MarkerSize', 5)
plot(x_online_unsafe(1,:),x_online_unsafe(2,:), '--', ...
    'Color', [0 0 1], 'Marker', 'x', 'LineWidth', 1.5, ...
    'MarkerIndices', 1:700:length(time), 'MarkerSize', 5)
plot(reference(1,:), reference(2,:),"Color","b","LineStyle","-","LineWidth",1)
plot(bound_x1, bound_x2, 'LineWidth',1); % Plot the boundary
xlabel("x1")
ylabel("x2")
legend("Safe Trajectory","unsafe trajectory","reference","boundary")

% save("z_online_snac_10-15_tracking4.mat")
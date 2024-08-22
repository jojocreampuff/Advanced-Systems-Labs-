clc; clear; close all;

% Define plant dynamics
Ix = 1;   % moment of inertia (kg*m^2)
Iy = 1;   % moment of inertia (kg*m^2)
Iz = 1;   % moment of inertia (kg*m^2)
Mass = 1; grav = 9.81;
dt = 0.001;

N_states = 12;
N_controls = 4;
t_f = 100;
N = t_f/dt;

% syms x1_s x2_s x3_s x4_s x5_s x6_s x7_s x8_s x9_s x10_s x11_s x12_s
% 
% % Define the symbolic version of x
% x_sym = [x1_s; x2_s; x3_s; x4_s; x5_s; x6_s; x7_s; x8_s; x9_s; x10_s; x11_s; x12_s];
% 
% % Define the symbolic version of phi
% phi_sym = phi(x_sym);
% 
% % Compute the Jacobian matrix of phi with respect to x
% GRAD_PHI_sym = jacobian(phi_sym, x_sym);

% state domains
X_max = 100; X_min = -100;
Y_max = 100; Y_min = -100;
Z_max = 100; Z_min = -100;

u_max = 20; u_min = -20;
v_max = 20; v_min = -20;
w_max = 20; w_min = -20;

PHI_max = pi/6; PHI_min = -pi/6;
THE_max = pi/6; THE_min = -pi/6;
PSI_max = pi; PSI_min = -pi;

p_max =  pi/8; p_min =  -pi/8;
q_max =  pi/8; q_min =  -pi/8;
r_max =  pi/10; r_min =  -pi/10;

% define max state values and controls to non-dim
x1_max = X_max;
x2_max = Y_max;
x3_max = Z_max;
x4_max = u_max;
x5_max = v_max;
x6_max = w_max;
x7_max = PHI_max;
x8_max = THE_max;
x9_max = PSI_max;
x10_max = p_max;
x11_max = q_max;
x12_max = r_max;
max_states = [x1_max; x2_max; x3_max; x4_max; x5_max; x6_max; x7_max; x8_max; x9_max; x10_max; x11_max; x12_max]*1.2;

Ft_max = 40; tx_max = 2; ty_max = 2; tz_max = 1;
max_controls = [Ft_max; tx_max; ty_max; tz_max]*2;

Q = diag([1,1,1,1,1,1,1,1,1,1,1,1]);
R =  diag([1,1,1,1]);

no_neurns_critic = length(phi(ones(12,1)));

alpha_1 = 100;
alpha_2 = 10;
F1 = 10 * eye(no_neurns_critic); 
F2 = 10 * eye(no_neurns_critic);

W1 = zeros(no_neurns_critic, N);
W2 = zeros(no_neurns_critic, N);
W1(:,1) = .01*randn(no_neurns_critic, 1);
W2(:,1) = .01*randn(no_neurns_critic, 1);

cut_off = 0.8 * t_f;

x = zeros(N_states,N);
xbar = zeros(N_states,N);
u2 = zeros(N_controls,N);
u2_bar = zeros(N_controls,N);

MAX_AMP = 0.5;
min_width= 0.05;
max_width = 0.1;

square_wave = zeros(N_controls,N);
for j=1:N_controls

    amplitudes = MAX_AMP*(-1 + 2*rand(1,  N)); 
    widths = min_width + (max_width-min_width).*rand(1, N); 

    for i = 1:N
        start_index= randi( [1,N]);
        end_index = min(start_index + round(widths(i) * N), N);
        square_wave(j, start_index:end_index) = amplitudes(i);
    end
end

for i = 1:N-1

    if mod(i,100) == 0
        fprintf("%i\n", i)
    end

    t= i*dt;
    xbar(:,i) = x(:,i)./max_states; % this is xbar now
    if i == 1
        x_current = xbar(:,i);
        x_prev = zeros(12,1);
        GRAD_PHI = grad_phi(@phi, x_current, x_prev, dt);
    else 
        x_current = xbar(:,i);
        x_prev = xbar(:,i-1);
        GRAD_PHI = grad_phi(@phi, x_current, x_prev, dt);
    end

    u_prob = square_wave(:,i);

    % x_val = xbar(:,i).';
    % GRAD_PHI = subs(GRAD_PHI_sym, {x1_s, x2_s, x3_s, x4_s, x5_s, x6_s, x7_s, x8_s, x9_s, x10_s, x11_s, x12_s}, num2cell(x_val));
    % GRAD_PHI = double(GRAD_PHI);

    u2(:,i)  = -0.5 * R^-1 * Full_g_bar(xbar(:,i),Mass,Ix,Iy,Iz,max_states, max_controls).' * GRAD_PHI.' * W2(:,i); % (39
    u2_bar(:,i) = u2(:,i)./max_controls; % u2 is now u2_bar
    % disp(u2_bar(:,1))

    xbar_next = Full_f_bar(xbar(:,i),grav,Ix,Iy,Iz, max_states) + Full_g_bar(xbar(:,i),Mass,Ix,Iy,Iz,max_states, max_controls )* u2_bar(:,i); 
    
    sigma_2 = GRAD_PHI * xbar_next; %N by 1

    D_1_bar = GRAD_PHI* Full_g_bar(xbar(:,i),Mass,Ix,Iy,Iz,max_states, max_controls) * R^-1 * Full_g_bar(xbar(:,i), Mass, Ix,Iy,Iz,max_states, max_controls).' *GRAD_PHI.'; %N by N
    
    m = sigma_2 / ( (sigma_2.' * sigma_2 +1)^2); %N by 1

    W1(:,i+1) = W1(:,i) - dt * alpha_1 * m * (sigma_2.' * W1(:,i) + xbar(:,i).' * Q * xbar(:,i) + u2_bar(:,i).' * R * u2_bar(:,i) ); %This solves for W1 iteratively just like ode23 in prob3.m 

    W2(:,i+1) = W2(:,i) - dt * alpha_2 * ( F2 * W2(:,i) - F1 *  W1(:,i) - 0.25 * D_1_bar * W2(:,i) * m.' * W1(:,i)); %This solves for W2 iteratively just like ode23 in prob3.m 

    if t < cut_off
        x(:,i+1) = x(:,i) + dt * (Full_f(x(:,i),grav,Ix,Iy,Iz) + Full_g(x(:,i),Mass,Ix,Iy,Iz) * (u_prob + u2(:,i)) ); %This solves for x iteratively just like ode23 in prob3.m 
    else
        x(:,i+1) = x(:,i) + dt * ( Full_f(x(:,i),grav,Ix,Iy,Iz) + Full_g(x(:,i),Mass,Ix,Iy,Iz) * u2(:,i) );
    end

    if any(isnan(x), 'all')
    disp('Divergance :(');
        break;
    end

end


figure(1)
plot((1:i) * dt,W1(:,1:i))
title("Weights")
ylabel('$W_1$', 'interpreter', 'latex')
xlabel('Time (sec)')

figure(2)
plot((1:i) * dt,W2(:,1:i))
title("NN Weights")
ylabel('$W_2$', 'interpreter', 'latex')
xlabel('Time (sec)')

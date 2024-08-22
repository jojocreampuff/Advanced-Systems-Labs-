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

Q = diag([1,1,1,1,1,1,1,1,1,1,1,1]);
R =  diag([1,1,1,1]);

no_neurns_critic = length(phi(ones(12,1)));

alpha_1 = 10;
alpha_2 = 10;
F1 = 100 * eye(no_neurns_critic); 
F2 = 100 * eye(no_neurns_critic);

W1 = zeros(no_neurns_critic, N);
W2 = zeros(no_neurns_critic, N);
W1(:,1) = .1*randn(no_neurns_critic, 1);
W2(:,1) = .1*randn(no_neurns_critic, 1);
cut_off = 0.8 * t_f;


x = zeros(N_states,N);
u2 = zeros(N_controls,N);

MAX_AMP = 0.5;
min_width= 0.05;
max_width = 0.1;

square_wave = zeros(N_controls,N);
for j = 1:N_controls

    amplitudes = MAX_AMP*(-1 + 2*rand(1,  N)); 
    widths = min_width + (max_width-min_width).*rand(1, N); 

    for i = 1:N
        start_index= randi( [1,N]);
        end_index = min(start_index + round(widths(i) * N), N);
        square_wave(j, start_index:end_index) = amplitudes(i);
    end
   
end
square_wave(1,:) = square_wave(1,:) + .5;

for i = 1:N-1

    if mod(i,100) == 0
        fprintf("%i\n", i)
    end

    t= i*dt;

    u_prob = square_wave(:,i);
    if i == 1
        x_current = x(:,i);
        x_prev = zeros(12,1);
        GRAD_PHI = grad_phi(@phi, x_current, x_prev, dt);
    else 
        x_current = x(:,i);
        x_prev = x(:,i-1);
        GRAD_PHI = grad_phi(@phi, x_current, x_prev, dt);
    end

    u2(:,i)  = -0.5 * R^-1 * Full_g(x(:,i),Mass,Ix,Iy,Iz).' * GRAD_PHI.' * W2(:,i); % (39

    x_next = Full_f(x(:,i),grav,Ix,Iy,Iz) + Full_g(x(:,i),Mass,Ix,Iy,Iz)* u2(:,i); 

    sigma_2 = GRAD_PHI * x_next; %N by 1

    D_1_bar = GRAD_PHI* Full_g(x(:,i),Mass,Ix,Iy,Iz) * R^-1 * Full_g(x(:,i), Mass,Ix,Iy,Iz).' *GRAD_PHI.'; %N by N
    
    m = sigma_2 / ( (sigma_2.' * sigma_2 +1)^2); %N by 1

    W1(:,i+1) = W1(:,i) - dt * alpha_1 * m * (sigma_2.' * W1(:,i) + x(:,i).' * Q * x(:,i) + u2(:,i).' * R * u2(:,i) ); %This solves for W1 iteratively just like ode23 in prob3.m 

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

% 
% fprintf("mae W1 is %f ",mae(W1(:,end)-W1_prev))
% fprintf("mae W2 is %f\n",mae(W2(:,end)-W2_prev))
% W1_prev = W1(:,end);
% W2_prev = W2(:,end);

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

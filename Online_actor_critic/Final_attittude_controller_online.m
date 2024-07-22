clc; clear; close;

dt = 0.001;
grav = 9.81;

Ix = 0.3;   % moment of inertia (kg*m^2)
Iy = 0.4;   % moment of inertia (kg*m^2)
Iz = 0.5;   % moment of inertia (kg*m^2)

f=@(x) [(x(4,1) + x(5,1)*(sin(x(1,1))*tan(x(2,1))) + x(6,1)*(cos(x(1,1))*tan(x(2,1))));
                   (x(5,1)*cos(x(1,1)) - x(6,1)*sin(x(1,1)));
                   (x(5,1)*sin(x(1,1))/cos(x(2,1)) + x(6,1)*cos(x(1,1))/cos(x(2,1)));
                   ((Iy - Iz) / Ix * x(5,1) * x(6,1));
                   ((Iz - Ix) / Iy * x(4,1) * x(6,1));
                   ((Ix - Iy) / Iz * x(4,1) * x(5,1))];

g = @(x) [0 0 0; 0 0 0; 0 0 0; 1/Ix 0 0; 0 1/Iy 0; 0 0 1/Iz];

N_states = 6;
N_controls = 3; 

t_f = 100;
cut_off = 0.8 * t_f;

N = t_f/dt;
x = zeros(N_states,N);
u2 = zeros(N_controls,N);

% Q = diag([1e6,1e6,1e5,1e5,1e5,1e5]);
% R = diag([0.5e4,0.5e4,0.5e4]);
% 
Q = diag([10000,10000,10000,1000,1000,1000]);
R =  diag([1,1,1]);

MAX_AMP = 0.01;
min_width= 0.05;
max_width = 0.1;
square_wave = zeros(N_controls,N);

for j=1:N_controls

    amplitudes = MAX_AMP*(-1 + 2*rand(1,  N)); % Random amplitudes between -1 and 1
    widths = min_width + (max_width-min_width).*rand(1, N); % Random widths between 0.01 and 0.1

    for i = 1:N
        start_index= randi( [1,N]);
        end_index = min(start_index + round(widths(i) * N), N);
        square_wave(j, start_index:end_index) = amplitudes(i);
    end
end

phi = @(x) [1
    x(1,1).^2
    x(2,1).^2
    x(3,1).^2
    x(4,1).^2
    x(5,1).^2
    x(6,1).^2
    x(1,1).*x(2,1)
    x(1,1).*x(3,1)
    x(1,1).*x(4,1)
    x(1,1).*x(5,1)
    x(1,1).*x(6,1)
    x(2,1).*x(3,1)
    x(2,1).*x(4,1)
    x(2,1).*x(5,1)
    x(2,1).*x(6,1)
    x(3,1).*x(4,1)
    x(3,1).*x(5,1)
    x(3,1).*x(6,1)
    x(4,1).*x(5,1)
    x(4,1).*x(6,1)
    x(5,1).*x(6,1)
    ];

no_neurns_critic = length(phi([1,1,1,1,1,1]'));

grad_phi =@(x) [ ...
    0, 0, 0, 0, 0, 0;
    2*x(1,1),    0,    0,    0,    0,    0
    0, 2*x(2,1),    0,    0,    0,    0
    0,    0, 2*x(3,1),    0,    0,    0
    0,    0,    0, 2*x(4,1),    0,    0
    0,    0,    0,    0, 2*x(5,1),    0
    0,    0,    0,    0,    0, 2*x(6,1)
    x(2,1),   x(1,1),    0,    0,    0,    0
    x(3,1),    0,   x(1,1),    0,    0,    0
    x(4,1),    0,    0,   x(1,1),    0,    0
    x(5,1),    0,    0,    0,   x(1,1),    0
    x(6,1),    0,    0,    0,    0,   x(1,1)
    0,   x(3,1),   x(2,1),    0,    0,    0
    0,   x(4,1),    0,   x(2,1),    0,    0
    0,   x(5,1),    0,    0,   x(2,1),    0
    0,   x(6,1),    0,    0,    0,   x(2,1)
    0,    0,   x(4,1),   x(3,1),    0,    0
    0,    0,   x(5,1),    0,   x(3,1),    0
    0,    0,   x(6,1),    0,    0,   x(3,1)
    0,    0,    0,   x(5,1),   x(4,1),    0
    0,    0,    0,   x(6,1),    0,   x(4,1)
    0,    0,    0,    0,   x(6,1),   x(5,1)
    ];


alpha_1 = 50;
alpha_2 = 15;
F1 = 10 * eye(no_neurns_critic); 
F2 = 10 * eye(no_neurns_critic); 

W1 = ones(no_neurns_critic, N);
W2 = rand(no_neurns_critic, N);
W1_prev = W1(:,end);
W2_prev = W2(:,end);

epoch = 10;
for j = 1:epoch

    x(:,1) = zeros(N_states,1); %+ [(j-1)*.01*randn(N_states/2,1); (j-1)/(j+1)*.01*randn(N_states/2,1)];

    W1(:,1) = W1_prev;
    W2(:,1) = W2_prev;

    for i = 1:N-1
    
        t= i*dt;
    
        u_prob = square_wave(:,i);
    
        GRAD_PHI = grad_phi(x(:,i));
    
        u2(:,i)  = -0.5 * R^-1 * g(x(:,i)).' * GRAD_PHI.' * W2(:,i);
    
        f_plus_gu = f(x(:,i)) + g(x(:,i))* u2(:,i); 
    
        sigma_2 = GRAD_PHI * f_plus_gu; %N by 1
    
        D_1_bar = GRAD_PHI* g(x(:,i)) * R^-1 * g(x(:,i)).' *GRAD_PHI.'; %N by N
        
        m = sigma_2 / ( (sigma_2.' * sigma_2 +1)^2); %N by 1
        
        W1(:,i+1) = W1(:,i) - dt * alpha_1 * m * (sigma_2.' * W1(:,i) + x(:,i).' * Q * x(:,i) + u2(:,i).' * R * u2(:,i) ); 
    
        W2(:,i+1) = W2(:,i) - dt * alpha_2 * ( F2 * W2(:,i) - F1 *  W1(:,i) - 0.25 * D_1_bar * W2(:,i) * m.' * W1(:,i)); 
    
        if t <cut_off
    
            x(:,i+1) = x(:,i) + dt * (f(x(:,i)) + g(x(:,i)) * (u_prob + u2(:,i)) );
        else
            x(:,i+1) = x(:,i) + dt * ( f(x(:,i)) + g(x(:,i)) * u2(:,i) );
        end
    
    end

    if any(isnan(x), 'all')
        disp('Divergance');
        break;
    end

    % printf("mae W1 is %f",mae(W1(:,end)-W1_prev))
    % printf("mae W2 is %f",mae(W2(:,end)-W2_prev))

    W1_prev = W1(:,end);
    W2_prev = W2(:,end);
end


W2_final_att = W2(:, i);

x_online = randn(N_states,1);
t_f_online = 10;
N_online = t_f_online/dt;

for i = 1:N_online-1
    u_online(:,i) = -0.5 * R^-1 * g(x_online(:,i)).' * grad_phi(x_online(:,i)).' * W2_final_att;

    x_online(:, i+1) = x_online(:,i) + dt *( f(x_online(:,i)) + g(x_online(:,i))* u_online(:,i));
end

figure(1);
plot((1:N) * dt, x)
title("Attitude States", 'Interpreter', 'latex')
legend('$\phi$','$\theta$', '$\psi$', '$\dot{\phi}_b$', '$\dot{\theta}_b$', '$\dot{\psi}_b$', 'Interpreter', 'latex')
ylabel('States', 'Interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(2)
plot((1:N) * dt,W1)
title("Weights")
ylabel('$W_1$', 'interpreter', 'latex')
xlabel('Time (sec)')

figure(3)
plot((1:N) * dt,W2)
title("NN Weights")
ylabel('$W_2$', 'interpreter', 'latex')
xlabel('Time (sec)')

figure(4)
plot((1:N_online) * dt,x_online)

ylabel('$x_{online}$', 'interpreter', 'latex')
xlabel('Time (sec)')

figure(5)
plot((1:N_online-1) * dt,u_online)

ylabel('$u_{online}$', 'interpreter', 'latex')
xlabel('Time (sec)')


figure(6)
plot((1:N) * dt,square_wave)
title("Excitation Wave")
ylabel('$u_{probing}$', 'interpreter', 'latex')
xlabel('Time (sec)')


% save("attitude_workspace_online.mat")
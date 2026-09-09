clc
clear
close all

dt = 0.01;
dx_1 = 0.05;
dx_2 = 0.05;
du_1 = 0.05;

x_1_min = -2;
x_1_max = 2;

x_2_min = -2;
x_2_max = 2;

u_1_min = -10;
u_1_max = 10;

t_0 = 0;
t_f = 10+dt;


% these are discrete cost function parameters
% Q_dt = dt*Q_ct
Q = diag([100,10])*dt;
S = diag([100,10])*dt;
R = 1*dt;

last_row = 3;

s11 = S(1,1);
s12 = S(1,2);
s21 = S(2,1);
s22 = S(2,2);

q11 = Q(1,1);
q12 = Q(1,2);
q21 = Q(2,1);
q22 = Q(2,2);

r_max = 3;
x_max = 3;

f_11 = @(x_1,x_2) x_2;
f_12 = @(x_1,x_2) (1 - x_1.^2 ) .* x_2 - x_1;

g_11 = @(x_1,x_2) 0;
g_12 = @(x_1,x_2) 1;

% f_r_1 = @(t) [(1/pi)*sin(.5*pi*t);    .5*cos(0.5*pi*t)];

F_11 = @(x_1,x_2) x_1 + dt * f_11(x_1,x_2);
F_12 = @(x_1,x_2) x_2 + dt * f_12(x_1,x_2);

G_11 = @(x_1,x_2) dt * g_11(x_1,x_2);
G_12 = @(x_1,x_2) dt * g_12(x_1,x_2);

% F_r_1 = @(r_1,r_2, t) [r_1; r_2] + dt * f_r_1(t);

N = (t_f - t_0 )/dt;


for i = 1:N
    % r(:,i+1) =  F_r_1(r(1,i), r(2,i), i * dt);
    r(:,i) = ref_refdot(i*dt);
end

[X1,X2,U1] = ndgrid(x_1_min:dx_1:x_1_max, x_2_min:dx_2:x_2_max, u_1_min:du_1:u_1_max);

No_Divisions_X1 = length(X1);
No_Divisions_X2 = length(X2);
No_Divisions_U1 = length(U1);

Best_Cost = 1e10;
count = 0;
j = 1;

r_N_1 = r(1,N);
r_N_2 = r(2,N);

tic

X_Next_1 = F_11(X1,X2) + G_11(X1, X2) .* U1;
X_Next_2 = F_12(X1,X2) + G_12(X1, X2) .* U1;

cost_temp = (s11*(r_N_1 - X_Next_1) + s21*(r_N_2 - X_Next_2)).*(r_N_1 - X_Next_1) + (s12*(r_N_1 - X_Next_1) + s22*(r_N_2 - X_Next_2)).*(r_N_2 - X_Next_2);

Best_Cost = zeros(length(cost_temp(1,:,1)), length(cost_temp(:,1,1)), N);
Best_Control = zeros(length(cost_temp(1,:,1)), length(cost_temp(:,1,1)),N-1);

fprintf('Backward calculations for time = %g (sec) \n ', N *dt)

for i = 1: length(cost_temp(1,:,1))
    for j = 1: length(cost_temp(:,1,1))
        [best_cost, index]  = min(cost_temp(i,j,:));
        Best_Control(i,j,N-1) = U1(i,j, index);
        Best_Cost(i,j,N) = best_cost;
    end
end

for k = N-2:-1:1
    
    if mod(k, 50)==0
        fprintf('Backward calculations for time = %g (sec) \n ', k *dt)
    end
    
    Best_Cost_old = Best_Cost(:,:, k+2); % makes 301th entery
    Best_Control_old = Best_Control(:,:, k+1); % makes 300th entery
    cost_temp = zeros(length(X1(:,1,1)), length(X1(1,:,1)), length(X1(1,1,:)));
     
    r_k_1 = r(1,k+1);
    r_k_2 = r(2,k+1);
    
    In_Resion_Logic = (X_Next_1>= x_1_min) .* (X_Next_1 <= x_1_max) ...
        .* (X_Next_2 >= x_2_min) .* (X_Next_2 <= x_2_max);
    
    Inside_Region = find(In_Resion_Logic);
    
    Outside_Region = find(~In_Resion_Logic);    
    
    cost_temp(Outside_Region) = 1e20;
    
    if isempty(Inside_Region)~= 1
        cost_temp(Inside_Region) = (q11*dt*(r_k_1 - X_Next_1(Inside_Region)) + ...
            q21*dt*(r_k_2 - X_Next_2(Inside_Region))).*...
            (r_k_1 - X_Next_1(Inside_Region)) + ...
            (q12*dt*(r_k_1 - X_Next_1(Inside_Region)) + ...
            q22*dt*(r_k_2 - X_Next_2(Inside_Region))).*...
            (r_k_2 - X_Next_2(Inside_Region)) + ...
            R*dt * U1(Inside_Region).^2 + ...
            interpn(X1(:,:,1), X2(:,:,1), Best_Cost_old, ...
            X_Next_1(Inside_Region), X_Next_2(Inside_Region));
    end
    for i = 1: length(cost_temp(1,:,1))
        for j = 1: length(cost_temp(:,1,1))
            [best_cost, index]  = min(cost_temp(i,j,:));
            Best_Control(i,j,k) = U1(i,j, index);
            Best_Cost(i,j,k+1) = best_cost;
        end
    end
    
end

Elapsed_Time_DP = toc;

fprintf('\n*********************************************\n')
fprintf('*********************************************\n')
fprintf('******* DONE WITH DYNAMIC PROGRAMMING *******\n')
fprintf('*********************************************\n')
fprintf('*********************************************\n')


fprintf('\n****** Forward Propagation Starts Now *******\n')

tic
x_DP(:, 1) = [0; 0];
x_DP2(:,1) = [1;1];
x_DP3(:,1) = [-1;-1];
x_DP4(:,1) = [-2;-2];

time(1) = 0;
u_DP = zeros(1,N-1);
u_DP2 = zeros(1,N-1);
u_DP3 = zeros(1,N-1);
u_DP4 = zeros(1,N-1);
time = zeros(1,N);
for k = 1 : N-1

    u_DP(k) = interpn(X1(:,:,1), X2(:,:,1), Best_Control(:,:,k), x_DP(1,k), x_DP(2,k));
    x_DP(1,k+1) = F_11(x_DP(1,k),x_DP(2,k)) + G_11(x_DP(1,k),x_DP(2,k)) * u_DP(k);
    x_DP(2,k+1) = F_12(x_DP(1,k),x_DP(2,k)) + G_12(x_DP(1,k),x_DP(2,k)) * u_DP(k);
    
    u_DP2(k) = interpn(X1(:,:,1), X2(:,:,1), Best_Control(:,:,k), x_DP2(1,k), x_DP2(2,k));
    x_DP2(1,k+1) = F_11(x_DP2(1,k),x_DP2(2,k)) + G_11(x_DP2(1,k),x_DP2(2,k)) * u_DP2(k);
    x_DP2(2,k+1) = F_12(x_DP2(1,k),x_DP2(2,k)) + G_12(x_DP2(1,k),x_DP2(2,k)) * u_DP2(k);

    u_DP3(k) = interpn(X1(:,:,1), X2(:,:,1), Best_Control(:,:,k), x_DP3(1,k), x_DP3(2,k));
    x_DP3(1,k+1) = F_11(x_DP3(1,k),x_DP3(2,k)) + G_11(x_DP3(1,k),x_DP3(2,k)) * u_DP3(k);
    x_DP3(2,k+1) = F_12(x_DP3(1,k),x_DP3(2,k)) + G_12(x_DP3(1,k),x_DP3(2,k)) * u_DP3(k);

    u_DP4(k) = interpn(X1(:,:,1), X2(:,:,1), Best_Control(:,:,k), x_DP4(1,k), x_DP4(2,k));
    x_DP4(1,k+1) = F_11(x_DP4(1,k),x_DP4(2,k)) + G_11(x_DP4(1,k),x_DP4(2,k)) * u_DP4(k);
    x_DP4(2,k+1) = F_12(x_DP4(1,k),x_DP4(2,k)) + G_12(x_DP4(1,k),x_DP4(2,k)) * u_DP4(k);    

    if mod(k,50) ==0
        fprintf('Forward propagation for time = %g (sec) \n', k*dt)
    end
%     x_DP(1,k+1) = x1_for(x_DP(1,k), x_DP(2,k), u_DP(k));
%     x_DP(2,k+1) = x2_for(x_DP(1,k), x_DP(2,k), u_DP(k));
    time(k+1) = k*dt;
end

Elapsed_time_running_DP = toc;

fprintf('\nElapsed time Deriving DP = %g (sec)\n', Elapsed_Time_DP)
fprintf('Elapsed time Using DP in Forward Propagation= %g (sec)\n', Elapsed_time_running_DP)

subplot(3,1,1); plot(time, x_DP(1, 1:length(time)), 'k', time, r(1, 1:length(time)), 'r--')
xlabel('Time (sec)' )
ylabel('States')

subplot(3,1,2); plot(time, x_DP(2, 1:length(time)), 'k', time, r(2, 1:length(time)), 'r--')
xlabel('Time (sec)' )
ylabel('States')

subplot(3,1,3); plot(time(1:length(u_DP)), u_DP)
xlabel('Time (sec)' )
ylabel('u')

save("DP_vanderpol_tracking.mat")
% subplot(2,1,1); plot(time, x_DP(2, 1:length(time)), 'k', time, r(2, 1:length(time)), 'r--', ...
%     time, x_DP2(2, 1:length(time)), ...
%     time, x_DP3(2, 1:length(time)), ...
%     time, x_DP4(2, 1:length(time)))
% xlabel('Time (sec)' )
% ylabel('States')
% 
% subplot(2,1,2); plot(time(1:length(u_DP)), u_DP, ...
%     time(1:length(u_DP)), u_DP2, ...
%     time(1:length(u_DP)), u_DP3, ...
%     time(1:length(u_DP)), u_DP4)
% xlabel('Time (sec)' )
% ylabel('u')
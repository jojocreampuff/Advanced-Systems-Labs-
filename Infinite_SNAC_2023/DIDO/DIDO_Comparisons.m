clc; clear all; close all;

load('DIDO_workspace_SNAC_Analog.mat','t','u_1','u_2','u_3','u_4')
addpath('..\functions')

% Constants
grav = 9.81;
m    = 1;
Ix   = 0.3;
Iy   = 0.4;
Iz   = 0.5;

% Full Dynamics 2.25 of Sabation Quadrotor Thesis
Full_F = @(x,grav,Ix,Iy,Iz,dt) x + dt * Full_f_225(x,grav,Ix,Iy,Iz); % drift dynamics
Full_G = @(x,m,Ix,Iy,Iz,dt) dt * Full_g_225(x,m,Ix,Iy,Iz); % control dynamics

N = length(t);

u_noise = zeros(4,N-1);
x = [zeros(12,1)];

noise = 0;
u = [u_1; u_2; u_3; u_4];
for i = 1:N-1
    u_noise(:,i) = u(:,i) .* (1 + noise.*(2*rand(size(u(:,i))) - 1)); % add randomness

    dt = t(i+1)-t(i);
    % Passing controls though drone dynamics
    x(:, i+1) = Full_F(x(:,i),grav,Ix,Iy,Iz,dt) + Full_G(x(:,i),m,Ix,Iy,Iz,dt) * u_noise(:,i);
end

%% Plotting
figure
subplot(4,1,1)
grid on
hold on
plot(t(1:79), u_noise(1,:), 'Linewidth', 1.5) %* FT
title('DIDO Quadcopter Controls')
ylabel('$f_t$ (N)','Interpreter','latex'), xlabel('time (s)')
ylim([9.7 10.1])
xlim([0 50])

subplot(4,1,2)
grid on
hold on
plot(t(1:79), u_noise(2,:), 'Linewidth', 1.5)
ylabel('$\tau_x$ (Nm)','Interpreter','latex'), xlabel('time (s)')
ylim([-4E-4 4E-4])
xlim([0 50])

subplot(4,1,3)
grid on
hold on
plot(t(1:79), u_noise(3,:), 'Linewidth', 1.5)
ylabel('$\tau_y$ (Nm)','Interpreter','latex'), xlabel('time (s)')
ylim([-4E-4 8E-4])
xlim([0 50])

subplot(4,1,4)
grid on
hold on
plot(t(1:79), u_noise(4,:), 'Linewidth', 1.5)
ylabel('$\tau_z$ (Nm)','Interpreter','latex'), xlabel('time (s)')
ylim([-1.5E-5 1.5E-5])
xlim([0 50])

figure
plot3(x_path(1:41)*X, y_path(1:41)*Y, -z_path(1:41)*Z, '--', x(1,1:41)*X, x(2,1:41)*Y, -x(3,1:41)*Z, 'Linewidth', 1.5)
grid on
hold on
title('3D Trajectory')
xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
legend('Reference Trajectory', 'Path 1', 'Location', 'northeast');


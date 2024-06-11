clc; clear all; close all;

load('DIDO_workspace_SNAC_Analog.mat','t','u_1','u_2','u_3','u_4')
addpath('..\functions')

figure
plot3(x_path(1:41)*X, y_path(1:41)*Y, -z_path(1:41)*Z, '--', x(1,1:41)*X, y(2,1:41)*Y, -z(3,1:41)*Z, 'Linewidth', 1.5)
grid on
hold on
title('3D Trajectory')
xlabel('x (m)'), ylabel('y (m)'), zlabel('z (m)')
legend('Reference Trajectory', 'Path 1', 'Location', 'northeast');

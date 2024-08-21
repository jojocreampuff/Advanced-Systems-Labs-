%% Plot stuff for the poster

clear; clc; close;

load("attitude_workspace_online.mat")
% load("position_workspace_online.mat")

figure(1);
plot((1:N) * dt, x)
title("Attitude States", 'Interpreter', 'latex')
legend('$\phi$','$\theta$', '$\psi$', '$\dot{\phi}_b$', '$\dot{\theta}_b$', '$\dot{\psi}_b$', 'Interpreter', 'latex','Orientation','horizontal')
ylabel('States', 'Interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(2)
plot((1:N) * dt,W1)
title("Weights", 'Interpreter', 'latex')
ylabel('$W_1$', 'interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(3)
plot((1:N) * dt,W2)
title("NN Weights $W_{2}$ (Attitude Controller)", 'Interpreter', 'latex')
ylabel('$W_{2}$', 'interpreter', 'latex')
ylim([.5 1.75])
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(4)
plot((1:N_online) * dt,x_online)
title("Attitude Controller Verification",'interpreter', 'latex')
ylabel('Attitude States', 'interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(5)
plot((1:N_online-1) * dt,u_online)

ylabel('$u_{online}$', 'interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')


figure(6)
plot((1:N) * dt,square_wave)
title("Excitation Wave", 'Interpreter', 'latex')
ylabel('$u_{probing}$', 'interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')

clear; clc;

load("position_workspace_online.mat")

figure(7);
plot((1:N) * dt, x)
title('Position States', 'Interpreter', 'latex')
legend('$x$', '$y$', '$z$', '$\dot{x}_b$', '$\dot{y}_b$', '$\dot{z}_b$', 'Interpreter', 'latex', 'Orientation','horizontal')
ylabel('States', 'Interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(8)
plot((1:N) * dt,W1)
title("Weights", 'Interpreter', 'latex')
ylabel('$W_1$', 'interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(9)
plot((1:N) * dt,W2)
title("NN Weights $W_{2}$ (Position Controller)", 'Interpreter', 'latex')
ylabel('$W_{2}$', 'interpreter', 'latex')
% ylim([.5 1.75])
xlabel('Time (sec)', 'Interpreter', 'latex')

figure(10)
plot((1:N_online) * dt,x_online)
title("Position Controller Verification",'interpreter', 'latex')
ylabel('Position States', 'interpreter', 'latex')
xlabel('Time (sec)', 'Interpreter', 'latex')


file_name = "att_states";


saveFigures(file_name);

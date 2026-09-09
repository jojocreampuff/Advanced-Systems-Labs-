clc
close all
clear

load('Attitude_R')
load('Attitude_W')
load('Attitude_G')
dt=.005;
t_end=20;
X=zeros(12,1);
U=[0;.01;.01;.01];
X_values=zeros(12,t_end/dt);
%% Create reference signal
for i= 1:t_end/dt

X=DroneDynamicsFunction(X,U,1,.3,.4,.5,dt);
X(7:12)=[-.0174;.0059;0;-.0616;.0761;0];
X_values(:,i+1)=X;
end


x=X_values(:,1);
for i= 1:t_end/dt
X_values(7:9,i)=X_values(7:9,i);
angles_ref(:,i) = [X_values(7:12,i)];

X_values(10:12,i)=X_values(10:12,i);
Attitude_error(:,i) = [x(7:12)] - angles_ref(:,i);
torques(:,i) = -Attitude_R^-1 * Attitude_G(Attitude_error(:,i))' * Attitude_W(:,:)' * Basis_Func_84(Attitude_error(:,i));
x=DroneDynamicsFunction(x,[0;torques(:,i)],1,.3,.4,.5,dt);

SNAC_controlled_x(:,i)=x;
end

% Adjusted time vector to match the state dimensions
time = 0:dt:(t_end-dt-1)/2;

figure;
for j = 7:9
    subplot(3, 1, j-6);
    plot(time, X_values(j,1:length(time)), 'b', 'LineWidth', 1.5); hold on;
    plot(time, SNAC_controlled_x(j,1:length(time)), 'r--', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel(['State ', num2str(j)]);
    legend('Reference', 'SNAC-Controlled');
    title(['State ', num2str(j)]);
end
sgtitle('Comparison of Reference and SNAC-Controlled States');
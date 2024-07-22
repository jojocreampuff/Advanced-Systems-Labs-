clc
clear
close all

dt = 0.001;
grav = 9.81;

Ix = 0.3;   % moment of inertia (kg*m^2)
Iy = 0.4;   % moment of inertia (kg*m^2)
Iz = 0.5;   % moment of inertia (kg*m^2)

load("position_workspace_online.mat")
load("attitude_workspace_online.mat")
W2_Attitude_Final = W2_final_att;
W2_Position_Final = W2_final;
% load("W2_Position_Final.mat")
% load("Workspace_Final_Attitude_Controller_Online_06_22_2024.mat", "W2_Attitude_Final")

r_psi = pi/6;

z_invert = -1;

% desired_path_1 = @(t) [(1-exp(-0.01*t))*9.81*cos(1.5*0.2*t);
%       (1-exp(-0.01*t))*9.81*sin(1.5*0.2*t);
%       z_invert*(.1*t + .1)];

% desired_path_1 = @(t)...
%             [9.81*cos(0.2*t);       % reference_x
%              9.81*sin(0.2*t);     % reference_y
%              z_invert*(3*cos(1*t) + 9.81)];      % reference_z

desired_path_1 = @(t)...
            [3*cos(1*t);  % reference_x
             -10*sin(0.1*t);  % reference_y
             z_invert* (-10*cos(0.1*t) + 11)];  

t = 0:dt:100-dt;

% Vectors initiations:
Path_Points_1 = zeros(3,length(t));
Path_Points_2 = zeros(3,length(t));

e_StatePosition_minus_Desired_Path = zeros(6,1);
u_xyz = zeros(3,1);
ft_r_Phi_r_Theta = zeros(3,1);
e_StateAngles_minus_DesiredAngle = zeros(6,1);
r_pqr = zeros(3,1);
r_PhiThetaPsi_pqr = zeros(6,1);
velocity_points = zeros(3,1);
r_PhiThetaPsi = zeros(3,1);
T_xyz = zeros(3,1);
Controls = zeros(4,1);

S = zeros(12,length(t));
S_dot = zeros(12,length(t));

S(:,1) = [0; -5; 0; 0; 0; 0; 0; 0; pi/4; 0; 0; 0]; 
S1(:,1) = [-5; 0; 0; 0; 0; 0; 0; 0; pi/4; 0; 0; 0];
S2(:,1) = [5; 0; 0; 0; 0; 0; 0; 0; pi/4; 0; 0; 0];
 
m = 3.2;
for i= 1:length(t)

    Path_Points_1(:,i) = desired_path_1(t(i)) ;

end

velocity_points_1 = gradient(Path_Points_1,0.001);

Pos_Vel_1 = [Path_Points_1
           velocity_points_1];

for i= 1:length(t)

    e_StatePosition_minus_Desired_Path(:,i) = S(1:6,i) - Pos_Vel_1(:,i); %first junktion box

    u_xyz(:,i) = Posi_Cont(e_StatePosition_minus_Desired_Path(:,i), W2_Position_Final) ;
    
    ft_r_Phi_r_Theta(:,i) = sys_solve(u_xyz(:,i), r_psi, m) ;

    r_PhiThetaPsi(:,i) = [ft_r_Phi_r_Theta(2:3,i);r_psi]; 
    
    diff_of_Angles(:,i) = deriv1(r_PhiThetaPsi,i,dt);                                                      
    r_pqr(:,i) = diff_of_Angles(:,i);
    r_PhiThetaPsi_pqr(:,i) = [r_PhiThetaPsi(:,i);r_pqr(:,i)];

    e_StateAngles_minus_DesiredAngle(:,i) = S(7:12,i) - r_PhiThetaPsi_pqr(:,i); %seconf junction

    T_xyz(:,i) = Atti_Cont(e_StateAngles_minus_DesiredAngle(:,i), Ix, Iy, Iz, W2_Attitude_Final);

    Controls(:,i) = [ft_r_Phi_r_Theta(1,i);T_xyz(:,i)];

    S_dot(:,i) = Full_f_225(S(:,i),grav,Ix,Iy,Iz) + Full_g_225(S(:,i),m,Ix,Iy,Iz) *  Controls(:,i);

    S(:,i+1) = S(:,i) + dt * S_dot(:,i);

    % if(S(3,i) < 0)
    %     S(3,i) = 0;
    % end

end

for i= 1:length(t)

    e_StatePosition_minus_Desired_Path(:,i) = S1(1:6,i) - Pos_Vel_1(:,i); %first junktion box

    u_xyz(:,i) = Posi_Cont(e_StatePosition_minus_Desired_Path(:,i), W2_Position_Final) ;

    % Solving for ft, phi and theta

    ft_r_Phi_r_Theta(:,i) = sys_solve(u_xyz(:,i), r_psi, m) ;

    r_PhiThetaPsi(:,i) = [ft_r_Phi_r_Theta(2:3,i);r_psi]; 
    
    diff_of_Angles(:,i) = deriv1(r_PhiThetaPsi,i,dt);                                                      
    r_pqr(:,i) = diff_of_Angles(:,i);
    r_PhiThetaPsi_pqr(:,i) = [r_PhiThetaPsi(:,i);r_pqr(:,i)];

    e_StateAngles_minus_DesiredAngle(:,i) = S1(7:12,i) - r_PhiThetaPsi_pqr(:,i); %seconf junction

    T_xyz(:,i) = Atti_Cont(e_StateAngles_minus_DesiredAngle(:,i), Ix, Iy, Iz, W2_Attitude_Final);

    Controls(:,i) = [ft_r_Phi_r_Theta(1,i);T_xyz(:,i)];

    S_dot_1(:,i) = Full_f_225(S1(:,i),grav,Ix,Iy,Iz) + Full_g_225(S1(:,i),m,Ix,Iy,Iz) *  Controls(:,i);

    S1(:,i+1) = S1(:,i) + dt * S_dot_1(:,i);

    % if(S1(3,i) < 0)
    %     S1(3,i) = 0;
    % end
end

for i= 1:length(t)

    e_StatePosition_minus_Desired_Path(:,i) = S2(1:6,i) - Pos_Vel_1(:,i); %first junktion box

    u_xyz(:,i) = Posi_Cont(e_StatePosition_minus_Desired_Path(:,i), W2_Position_Final) ;

    % Solving for ft, phi and theta

    ft_r_Phi_r_Theta(:,i) = sys_solve(u_xyz(:,i), r_psi, m) ;

    r_PhiThetaPsi(:,i) = [ft_r_Phi_r_Theta(2:3,i);r_psi]; 
    
    diff_of_Angles(:,i) = deriv1(r_PhiThetaPsi,i,dt);                                                      
    r_pqr(:,i) = diff_of_Angles(:,i);
    r_PhiThetaPsi_pqr(:,i) = [r_PhiThetaPsi(:,i);r_pqr(:,i)];

    e_StateAngles_minus_DesiredAngle(:,i) = S2(7:12,i) - r_PhiThetaPsi_pqr(:,i); %seconf junction

    T_xyz(:,i) = Atti_Cont(e_StateAngles_minus_DesiredAngle(:,i), Ix, Iy, Iz, W2_Attitude_Final);

    Controls(:,i) = [ft_r_Phi_r_Theta(1,i);T_xyz(:,i)];

    S_dot_2(:,i) = Full_f_225(S2(:,i),grav,Ix,Iy,Iz) + Full_g_225(S2(:,i),m,Ix,Iy,Iz) *  Controls(:,i);

    S2(:,i+1) = S2(:,i) + dt * S_dot_2(:,i);
    % 
    % if(S2(3,i) < 0)
    %     S2(3,i) = 0;
    % end

end

figure(1)
grid on
hold on
plot3(S(1,:),S(2,:),z_invert*S(3,:),'Color','g','LineWidth',1 )
plot3(S1(1,:),S1(2,:),z_invert*S1(3,:),'Color','b','LineWidth',1 )
plot3(S2(1,:),S2(2,:),z_invert*S2(3,:),'Color','c','LineWidth',1 )

plot3(Path_Points_1(1,:),Path_Points_1(2,:),z_invert*Path_Points_1(3,:),'--','LineWidth',2, "Color","r")
legend("Path 1", "Path 2","Path 3", "Reference", 'Interpreter', 'latex')
title('Tracking Simulation', 'Interpreter', 'latex')
xlabel("Meters", 'Interpreter', 'latex')
ylabel("Meters", 'Interpreter', 'latex')
zlabel("Meters", 'Interpreter', 'latex')
% zlim([0 10])
% annotation('test')


function uvw = discrete_deriv(x,dt)
    uvw = ones(size(x));
    uvw(:, 1) = (x(:, 2) - x(:, 1))/dt;
    for k = 2:(size(x, 2) - 1)
        uvw(:, k) = (x(:, k + 1) - x(:, k - 1))/(2*dt);
    end
    uvw(:, end) = (x(:, end) - x(:, end - 1))/dt;
end

function pqr = deriv(angles, i, dt)
    if i == 1
        pqr = ((angles(:, i) - 0)/dt);
    elseif i > 1
        pqr = ((angles(:, i) - angles(:, i - 1))/(dt));
    end
% Limit pqr to be between -5 and 5
    pqr(pqr > 5) = 5;
    pqr(pqr < -5) = -5;
end

function pqr = deriv1(angles, i, dt)
    if i == 1
        pqr = ((angles(:, i) - 0)/dt);
    elseif i==2
        pqr1_update = (angles(:,i) - angles(:, i-1))/dt; 
        pqr=pqr1_update;
    elseif i > 2
        pqr = ((angles(:, i) - angles(:, i - 2))/ (2*dt));
    end

    pqr(pqr > 5) = 5;
    pqr(pqr < -5) = -5;
end







clear; close all; clc;
%% This script finds the gradient "A" matrix of the non-dimensionalized attutude controller for SNAC

syms x1_bar x2_bar x3_bar x4_bar x5_bar x6_bar x1_max x2_max x3_max x4_max x5_max x6_max Ix Iy Iz dt u1_max u2_max u3_max

x_bar = [x1_bar; x2_bar; x3_bar; x4_bar; x5_bar; x6_bar]; % State variables
u_bar = [u1_max; u2_max; u3_max];

Attitude_f_bar = @(x_bar) [ (1/x1_max)*(x_bar(4)*x4_max + x_bar(5)*x4_max*(sin( x_bar(1)*x1_max )*tan( x_bar(2)*x2_max )) + x_bar(6)*x6_max*(cos( x_bar(1)*x1_max )*tan( x_bar(2)*x2_max )));
                            (1/x2_max)*(x_bar(5)*x5_max*cos( x_bar(1)*x1_max ) - x_bar(6)*x6_max*sin( x_bar(1)*x1_max ));
                            (1/x3_max)*(x_bar(5)*x5_max*sin( x_bar(1)*x1_max )/cos( x_bar(2)*x2_max ) + x_bar(6)*x6_max*cos( x_bar(1)*x1_max )/cos( x_bar(2)*x2_max ));
                            (1/x4_max)*((Iy - Iz) / Ix * x_bar(5) * x_bar(6)* x5_max* x6_max); 
                            (1/x5_max)*((Iz - Ix) / Iy * x_bar(4) * x_bar(6)* x4_max* x6_max);
                            (1/x6_max)*((Ix - Iy) / Iz * x_bar(4) * x_bar(5)* x4_max* x5_max)      ];

Attitude_g_bar = [0 0 0; 0 0 0; 0 0 0; u1_max/(Ix*x4_max) 0 0; 0 u2_max/(Iy*x5_max) 0; 0 0 u3_max/(Iz*x6_max)];

% Euler integration
Attitude_F_bar = @(x_bar) x_bar + dt * Attitude_f_bar(x_bar);
Attitude_G_bar = @(x_bar) Attitude_g_bar * dt;

xbar_k_plus_1 = Attitude_F_bar(x_bar) + Attitude_G_bar(x_bar) * u_bar;

output = jacobian(xbar_k_plus_1,x_bar);

row1 = output(1,:)
row2 = output(2,:)
row3 = output(3,:)
row4 = output(4,:)
row5 = output(5,:)
row6 = output(6,:)

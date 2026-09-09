%% Position controller
clear, clc
syms dt ux uy uz ux_max uy_max uz_max x1_bar x1_max x2_bar x2_max x3_bar x3_max x4_bar x4_max x5_bar x5_max x6_bar x6_max grav

x_bar = [x1_bar; x2_bar; x3_bar; x4_bar; x5_bar; x6_bar]; % State variables

Position_f_bar = @(x_bar) [x_bar(4)*x4_max/x1_max; x_bar(5)*x5_max/x2_max; x_bar(6)*x6_max/x3_max; 0; 0; grav/x6_max];
Position_g_bar = @(x_bar) [0 0 0; 0 0 0; 0 0 0; -ux_max/x4_max 0 0; 0 -uy_max/x5_max 0; 0 0 -uz_max/x6_max];


U_bar = [ux/ux_max; uy/uy_max; uz/uz_max];

Position_f = Position_f_bar(x_bar);

Position_g = Position_g_bar .* U_bar';

F = [
   x_bar(1) + dt * (Position_f(1) + dt * Position_g(1));
   x_bar(2) + dt * (Position_f(2) + dt * Position_g(2));
   x_bar(3) + dt * (Position_f(3) + dt * Position_g(3));
   x_bar(4) + dt * (Position_f(4) + dt * Position_g(4));
   x_bar(5) + dt * (Position_f(5) + dt * Position_g(5));
   x_bar(6) + dt * (Position_f(6) + dt * Position_g(6))
];

A = jacobian(F,x_bar)
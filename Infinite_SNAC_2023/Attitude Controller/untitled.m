%% Attitude controller
clear, clc
syms Ix Iy Iz dt tx ty tz tx_max ty_max tz_max x1_bar x1_max x2_bar x2_max x3_bar x3_max x4_bar x4_max x5_bar x5_max x6_bar x6_max

x_bar = [x1_bar; x2_bar; x3_bar; x4_bar; x5_bar; x6_bar]; % State variables

Attitude_f_bar = @(x_bar) [ (1/x1_max)*(x_bar(4)*x4_max + x_bar(5)*x4_max*(sin(x_bar(1)*x1_max)*tan(x_bar(2)*x2_max)) + x_bar(6)*x6_max*(cos(x_bar(1))*tan(x_bar(2))));
                            (1/x2_max)*(x_bar(5)*x5_max*cos(x_bar(1)*x1_max) - x_bar(6)*x6_max*sin(x_bar(1)*x1_max));
                            (1/x3_max)*(x_bar(5)*x5_max*sin(x_bar(1)*x1_max)/cos(x_bar(2)*x2_max) + x_bar(6)*x6_max*cos(x_bar(1)*x1_max)/cos(x_bar(2)*x2_max));
                            (1/x4_max)*((Iy - Iz) / Ix * x_bar(5) * x_bar(6)* x5_max* x6_max);
                            (1/x5_max)*((Iz - Ix) / Iy * x_bar(4) * x_bar(6)* x4_max* x6_max);
                            (1/x6_max)*((Ix - Iy) / Iz * x_bar(4) * x_bar(5)* x4_max* x5_max)      ];

Attitude_g_bar = @(x_bar) [0 0 0; 0 0 0; 0 0 0; tx_max/(Ix*x4_max) 0 0; 0 ty_max/(Iy*x5_max) 0; 0 0 tz_max/(Iz*x6_max)];


U_bar = [tx/tx_max; ty/ty_max; tz/tz_max];

Attitude_f = Attitude_f_bar(x_bar);

Attitude_g = Attitude_g_bar .* U_bar';

F = [
   x_bar(1) + dt * (Attitude_f(1) + dt * Attitude_g(1));
   x_bar(2) + dt * (Attitude_f(2) + dt * Attitude_g(2));
   x_bar(3) + dt * (Attitude_f(3) + dt * Attitude_g(3));
   x_bar(4) + dt * (Attitude_f(4) + dt * Attitude_g(4));
   x_bar(5) + dt * (Attitude_f(5) + dt * Attitude_g(5));
   x_bar(6) + dt * (Attitude_f(6) + dt * Attitude_g(6))
];

A = jacobian(F,x_bar);


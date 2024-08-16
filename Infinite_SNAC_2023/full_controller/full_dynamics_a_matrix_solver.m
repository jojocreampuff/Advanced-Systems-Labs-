%% Full dynamics A matrix solver
clear, clc
syms Ix Iy Iz dt grav m tx ty tz Ft tx_max ty_max tz_max Ft_max x1_bar x1_max x2_bar x2_max x3_bar x3_max x4_bar x4_max x5_bar x5_max...
    x6_bar x6_max x7_bar x7_max x8_bar x8_max x9_bar x9_max x10_bar x10_max x11_bar x11_max x12_bar x12_max

x_bar = [x1_bar; x2_bar; x3_bar; x4_bar; x5_bar; x6_bar; x7_bar; x8_bar; x9_bar; x10_bar; x11_bar; x12_bar]; % State variables
states_max = [x1_max; x2_max; x3_max; x4_max; x5_max; x6_max; x7_max; x8_max; x9_max; x10_max; x11_max; x12_max];

max_controls = [Ft_max; tx_max; ty_max; tz_max];
U_bar = [Ft/ Ft_max; tx/tx_max; ty/ty_max; tz/tz_max];

F = Full_f(x_bar,grav,Ix,Iy,Iz, states_max);

G = Full_g(x_bar,m,Ix,Iy,Iz, states_max, max_controls) .* U_bar';

Fx_gxu = [
   x_bar(1) + dt * (F(1) + dt * G(1));
   x_bar(2) + dt * (F(2) + dt * G(2));
   x_bar(3) + dt * (F(3) + dt * G(3));
   x_bar(4) + dt * (F(4) + dt * G(4));
   x_bar(5) + dt * (F(5) + dt * G(5));
   x_bar(6) + dt * (F(6) + dt * G(6));
   x_bar(7) + dt * (F(7) + dt * G(7));
   x_bar(8) + dt * (F(8) + dt * G(8));
   x_bar(9) + dt * (F(9) + dt * G(9));
   x_bar(10) + dt * (F(10) + dt * G(10));
   x_bar(11) + dt * (F(11) + dt * G(11));
   x_bar(12) + dt * (F(12) + dt * G(12))
];

A = jacobian(Fx_gxu,x_bar)
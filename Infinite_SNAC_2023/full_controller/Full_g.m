function g = Full_g(x_bar,m,Ix,Iy,Iz,max_states, max_controls)

x1_max = max_states(1);
x2_max = max_states(2);
x3_max = max_states(3);
x4_max = max_states(4);
x5_max = max_states(5);
x6_max = max_states(6);
x7_max = max_states(7);
x8_max = max_states(8);
x9_max = max_states(9);
x10_max = max_states(10);
x11_max = max_states(11);
x12_max = max_states(12);

Ft_max = max_controls(1);
tx_max = max_controls(2);
ty_max = max_controls(3);
tz_max = max_controls(4);

% assume these are all x_bar!!
x1 = x_bar(1);      x2 = x_bar(2);      x3 = x_bar(3);
x4 = x_bar(4);      x5 = x_bar(5);      x6 = x_bar(6);
x7 = x_bar(7);      x8 = x_bar(8);      x9 = x_bar(9);
x10 = x_bar(10);    x11 = x_bar(11);    x12 = x_bar(12);


g17 = -1/m * (sin(x7*x7_max)*sin(x9*x9_max) + cos(x7*x7_max)*cos(x9*x9_max)*sin(x8*x8_max));
g18 = -1/m * (-cos(x9*x9_max)*sin(x7*x7_max) + cos(x7*x7_max)*sin(x9*x9_max)*sin(x8*x8_max));
g19 = -1/m * (cos(x7*x7_max)*cos(x8*x8_max));


%    ft_bar                 tx_bar                           ty_bar                 tz_bar
g = [0                      0                               0                       0;
     0                      0                               0                       0;
     0                      0                               0                       0;
     (g17*Ft_max)/x4_max    0                               0                       0;
     (g18*Ft_max)/x5_max    0                               0                       0;
     (g19*Ft_max)/x6_max    0                               0                       0;
     0                      0                               0                       0;
     0                      0                               0                       0;
     0                      0                               0                       0;
     0                      tx_max/(Ix*x10_max)             0                       0;
     0                      0                               ty_max/(Iy*x11_max)     0;
     0                      0                               0                       tz_max/(Iz*x12_max)];

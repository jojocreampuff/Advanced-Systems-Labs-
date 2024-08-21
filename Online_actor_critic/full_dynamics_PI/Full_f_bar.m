function f_bar = Full_f_bar(x_bar,grav,Ix,Iy,Iz, max_states)

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

% assume these are all x_bar!!
x1 = x_bar(1);      x2 = x_bar(2);      x3 = x_bar(3);
x4 = x_bar(4);      x5 = x_bar(5);      x6 = x_bar(6);
x7 = x_bar(7);      x8 = x_bar(8);      x9 = x_bar(9);
x10 = x_bar(10);    x11 = x_bar(11);    x12 = x_bar(12);

f = [
     (x4*x4_max);
     (x5*x5_max);
     (x6*x6_max);
     (0);
     (0);
     (grav);
     ((x10*x10_max) + (x11*x11_max)*sin(x7*x7_max)*tan(x8*x8_max) + (x12*x12_max)*cos(x7*x7_max)*tan(x8*x8_max));
     ((x11*x11_max)*cos(x7*x7_max)         - (x12*x12_max)*sin(x7*x7_max));
     ((x11*x11_max)*sin(x7*x7_max)/cos(x8*x8_max) + (x12*x12_max)*cos(x7*x7_max)/cos(x8*x8_max));
     ((Iy - Iz) / Ix * (x11*x11_max) * (x12*x12_max));
     ((Iz - Ix) / Iy * (x10*x10_max) * (x12*x12_max));
     ((Ix - Iy) / Iz * (x10*x10_max) * (x11*x11_max))
     ];

f_bar = f./max_states; % i think this will work

end
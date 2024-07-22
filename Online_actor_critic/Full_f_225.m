function f = Full_f_225(X,grav,Ix,Iy,Iz)

x = X(1);  y = X(2);  z = X(3);
u = X(4);  v = X(5);  w = X(6);
ph = X(7); th = X(8); ps = X(9);
p = X(10); q = X(11); r = X(12);

f = [(u);
     (v);
     (w);
     (0);
     (0);
     (grav);
     ((p) + (q)*sin(ph)*tan(th) + (r)*cos(ph)*tan(th));
           ((q)*cos(ph)         - (r)*sin(ph));
           ((q)*sin(ph)/cos(th) + (r)*cos(ph)/cos(th));
     ((Iy - Iz) / Ix * (q) * (r));
     ((Iz - Ix) / Iy * (p) * (r));
     ((Ix - Iy) / Iz * (p) * (q))];
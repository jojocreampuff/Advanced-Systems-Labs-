function g = Full_g(X,m,Ix,Iy,Iz)


x = X(1);  y = X(2);  z = X(3);
u = X(4);  v = X(5);  w = X(6);
ph = X(7); th = X(8); ps = X(9);
p = X(10); q = X(11); r = X(12);


g17 = -1/m * (sin(ph)*sin(ps) + cos(ph)*cos(ps)*sin(th));
g18 = -1/m * (-cos(ps)*sin(ph) + cos(ph)*sin(ps)*sin(th));
g19 = -1/m * (cos(ph)*cos(th));


%    ft             tx              ty              tz
g = [0              0               0               0;
     0              0               0               0;
     0              0               0               0;
     g17            0               0               0;
     g18            0               0               0;
     g19            0               0               0;
     0              0               0               0;
     0              0               0               0;
     0              0               0               0;
     0              1/Ix            0               0;
     0              0               1/Iy            0;
     0              0               0               1/Iz];


end

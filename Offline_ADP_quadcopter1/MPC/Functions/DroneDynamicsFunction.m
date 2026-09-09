function X_new=DroneDynamicsFunction(X,U,m,Ix,Iy,Iz,dt)


g=9.81; 

%% functions
Full_f = @(X)[
    X(4);
    X(5);
    X(6);
    0;
    0;
    g;
    X(10)+X(11)*(sin(X(7))*tan(X(8)))+X(12)*(cos(X(8))*tan(X(8)));
    X(11)*(cos(X(7)))-X(12)*sin(X(7));
    X(11)*(sin(X(7))/cos(X(8)))+X(12)*cos(X(7))/cos(X(8));
    (Iy-Iz)/Ix*X(11)*X(12);
    (Iz-Ix)/Iy*X(10)*X(12);
    (Ix-Iy)/Iz*X(10)*X(11);];
Full_g = @(X) [
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    -1/m*(sin(X(7))*sin(X(9))+cos(X(7))*cos(X(9))*sin(X(8))) 0 0 0;
    -1/m*(cos(X(7))*sin(X(9))*sin(X(8))-cos(X(9))*sin(X(7))) 0 0 0;
    -1/m*(cos(X(7))*cos(X(8))) 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 0 0 0;
    0 1/Ix 0 0;
    0 0 1/Iy 0;
    0 0 0 1/Iz
    ];

Full_F = @(X) X + dt * Full_f(X); % discretized drift dynamics
Full_G = @(X) dt * Full_g(X);

%% Calculate new X


X_new=Full_F(X)+Full_G(X)*U;

end

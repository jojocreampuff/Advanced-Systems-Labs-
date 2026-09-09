function [accn_dist, alpha_dist] = wind_f_m(Wxyz, X, m, Ix, Iy, Iz)

    %% wind sim stuff
    Cd = [.8; 0.8;1.2]; % Drag coefficient
    L = 0.2; % Half arm length (m)
    h = 0.05;
    A = [L*2*h, L*2*h, 2*L*L]; % Approximate frontal areas [x, y, z] in m^2
    r = [ L,  L, -L, -L;   % X positions of motors
          L, -L, -L,  L;   % Y positions of motors
          0,  0,  0,  0 ]; % Z positions (motors at same height)
    rho = 1.225; % Air density (kg/m^3)

    R = eul2rotm_zyx( X(7), X(8), X(9) );
    Wxyz_body = R'*Wxyz;
    V_rel = X(4:6) - Wxyz_body;
    F_wind = -0.5 * rho * Cd .*diag(A) * (V_rel .* abs(V_rel)); % Compute wind forces
    
    F_wind_motors = repmat(F_wind, 1, 4) + 0.01*randn(3,4);

    M_wind_motors = cross(r, F_wind_motors, 1);
    M_wind = sum(M_wind_motors, 2);  
    
    accn_dist = F_wind/m;
    alpha_dist=    [M_wind(1)/Ix;
                    M_wind(2)/Iy;
                    M_wind(3)/Iz];
end
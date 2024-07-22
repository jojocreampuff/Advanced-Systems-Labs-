function out = sys_solve(ux_uy_uz, psi, m)
    out = zeros(3,1);
    a = sin(psi);
    b = cos(psi);

    ux = ux_uy_uz(1,:);
    uy = ux_uy_uz(2,:);
    uz = ux_uy_uz(3,:);
    
    A = ux / (uz - 9.81);
    B = uy / (uz - 9.81);
    C = (a*A - b*B) / (a^2 + b^2);
    
    roll = atan( (B + b*C) / a);
    pitch = atan (C * cos(roll));
    ft = ( m / (cos(pitch)*cos(roll)) ) * (9.81 - uz);
    out(1) = ft;
    out(2) = pitch;
    out(3) = roll;
    
end
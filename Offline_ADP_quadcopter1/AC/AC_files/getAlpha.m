function alpha = getAlpha(xDD, yDD, zDD)
    constants
    
    ft  = mass*(grav^2 - 2*grav.*zDD + xDD.^2 + yDD.^2 + zDD.^2).^(1/2);
    phi = asin(mass*yDD./ft);
    the = asin(-mass*xDD./(ft.*c(phi)));
    psi = zeros(size(xDD));


    p = getDDerivative(phi);
    q = getDDerivative(the);
    r = getDDerivative(psi);

    alpha = [ft; phi; the; psi; p; q; r];

end
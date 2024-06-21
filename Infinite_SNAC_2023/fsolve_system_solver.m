function out = fsolve_system_solver(uxyz,r_psi, m)
    g = 9.81;
        % F(x) = 0
    eqns = @(vars) [uxyz(1) + vars(1)/m.*( sin(vars(2)).*sin(r_psi) + cos(vars(2)).*cos(r_psi).*sin(vars(3)) );...
                    uxyz(2) + vars(1)/m.*( cos(vars(2)).*sin(r_psi).*sin(vars(3)) - cos(r_psi).*sin(vars(2)) );...
                    uxyz(3) - g + vars(1)/m.*(cos(vars(2)).*cos(vars(3)))];
        % vars(1) = ft
        % vars(2) = phi
        % vars(3) = theta

        % initial guess for the solution
        x0 = [1; pi/4; pi/6];
        options = optimset('Display','off');

    for i = 1:5
        
        
        % solve the system of equations
        
        
        out = fsolve(eqns, x0, options);

        x0 = out;

    end
end
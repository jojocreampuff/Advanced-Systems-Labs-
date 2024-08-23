function out = excitation(N_controls, cutoff, dt)

    N = cutoff/dt;
    out = zeros(4,N+1);


    for i = 1:N_controls
        a = 10*randn(1,1);
        b = randn(1,1);
        c = randn(1,1);
        d = randn(1,1);
        for j = 1:N+1
            t = j*dt;
            out(i,j) = .1*exp(-.001)* ( sin(2*a*t)^2 *cos(a*t) + sin(3*a*t)^2 *cos(1.5*a*t) + sin(4*a*t)^2 *cos(2*a*t) + sin(8*a*t)^2 *cos(4*a*t) + sin(2*b*t)^2 *cos(a*t) + sin(3*b*t)^2 *cos(1.5*b*t) + sin(4*b*t)^2 *cos(2*b*t) + sin(8*b*t)^2 *cos(4*b*t) + sin(2*c*t)^2 *cos(c*t) + sin(3*c*t)^2 *cos(1.5*c*t) + sin(4*c*t)^2 *cos(2*c*t) + sin(8*c*t)^2 *cos(4*c*t) + sin(2*d*t)^2 *cos(d*t) + sin(3*d*t)^2 *cos(1.5*d*t) + sin(4*d*t)^2 *cos(2*d*t) + sin(8*d*t)^2 *cos(4*d*t));
        end
    end
    out(1,:) = out(1,:)+1;
end



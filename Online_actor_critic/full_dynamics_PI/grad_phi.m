function out = grad_phi(phi, x, x_prev, dt)

    phi_x = phi(x);           % n x 1 vector (current state)
    phi_x_prev = phi(x_prev); % n x 1 vector (previous state)
    
    n = length(phi_x); 
    out = zeros(n, length(x));

    for i = 1:length(x)
        delta_x = zeros(size(x));
        delta_x(i) = x(i) - x_prev(i);  
        
        if abs(delta_x) < eps
            out(:, i) = 0;
        else
            out(:, i) = (phi_x - phi_x_prev) / delta_x(i);
        end
    end

end

function out = online_snac_training_tracking(t,initials)
    global Q R alpha N_states N_neurons tf amp_mult N_R theta_star u_max
%% extract inputs
    x = initials(1:N_states);
    theta = initials(N_states+1:end);
    W = reshape(theta, N_neurons,N_states+N_R);   
    %% reference and error
    [r, r_dot] = ref_refdot(t);
    e = x - r;
    phi = PHI(e, r);
    Z = [e;r];

    %% dynamics
    f =[x(2); (1-x(1).^2).*x(2) - x(1)]; %f(x) = f(e + r)
    g =[ 0 ;  1];

    F_z = [f - r_dot
            r_dot];
    G_z = [g;
            0*g];
    %% control 
    u = -0.5*R^-1 * G_z' * W'*phi;
    %% Noisey signal 
    u_prob = u_excite(t,amp_mult);
    
    % exploration step
    if t<=(tf*.9)
        unew = u + u_prob;
        unew = max(min(unew, u_max), -u_max);
        z_dot = F_z + G_z*unew;
        Q_z = [Q , zeros(N_states,N_states);
                zeros(N_states,N_states), zeros(N_states,N_states)];
    
        r_x_u = Z'*Q_z*Z + unew'*R*unew;
        %% update law
        sigma_z = [z_dot(1)*phi; z_dot(2)*phi; z_dot(3)*phi; z_dot(4)*phi];    % sigma = z_dot kron phi
        norm_factor = 1 / ((1 + sigma_z'*sigma_z))^2;
        N = kron(eye(2*N_states), phi');
        M = G_z*R^-1*G_z';
    
        F1 = .5*N'*M*N*theta_star - .5*sigma_z*norm_factor*theta_star'*N'*M*N*theta_star + ...
            0.25*sigma_z*norm_factor*theta_star'*N'*M*N*theta_star - norm_factor*(sigma_z*sigma_z')*theta_star;
        F2 = norm_factor*(sigma_z*sigma_z');
    
        A = alpha^-1 * (F1 + F2*theta + 0.25*sigma_z*norm_factor*theta'*N'*M*N*theta);
    
        theta_increment = -alpha*norm_factor*sigma_z*(theta'*sigma_z + r_x_u) + A;

    else
        unew = u;
        theta_increment = 0*theta;
    end

    x_next = f + g*unew;
    
    % cat the output
    out = [x_next; theta_increment];
end
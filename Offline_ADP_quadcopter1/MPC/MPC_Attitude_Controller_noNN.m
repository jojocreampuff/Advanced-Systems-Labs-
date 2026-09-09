function [att_controls, J_current] = MPC_Attitude_Controller_noNN( ...
    current_reference, X_att, att_controls, N, J, JN, dt, K, decay_rate, alpha, epsilon)

% MPC attitude controller (finite difference gradient descent)
% Dynamics: x_dot = att_F(x) + att_G(x)*u
%
% State: X_att = [phi; theta; psi; p; q; r]  (6x1)
% Control: att_controls(:,i) = [tau_phi; tau_theta; tau_psi]  (3xN)

    % Receding horizon shift
    att_controls(:,1:N-1) = att_controls(:,2:N);

    for k = 1:K

        % ---- Forward simulate with current control sequence ----
        x = X_att(:);
        State_horizon = zeros(6, N);

        for i = 1:N
            u = att_controls(:, i);

            x_dot = att_F(x) + att_G(x) * u;
            x = x + x_dot * dt;

            State_horizon(:, i) = x;
        end

        % Cost for unperturbed controls
        original_cost = J([State_horizon; att_controls], current_reference) + ...
                        JN(State_horizon(:, N), current_reference(:, N));

        % ---- Finite difference gradient ----
        gradient = zeros(3, N);

        for n = 1:N
            for j = 1:3

                % Perturb one control component at one time
                perturbed_controls = att_controls;
                perturbed_controls(j, n) = perturbed_controls(j, n) + epsilon;

                % Start from the previous predicted state (saves work)
                if n > 1
                    x_p = State_horizon(:, n-1);
                else
                    x_p = X_att(:);
                end

                % Copy earlier horizon states (unchanged before n)
                p_State_horizon = State_horizon;
                if n > 1
                    p_State_horizon(:, 1:n-1) = State_horizon(:, 1:n-1);
                end

                % Re-simulate from n..N with perturbed controls
                for i = n:N
                    u_p = perturbed_controls(:, i);

                    x_p_dot = att_F(x_p) + att_G(x_p) * u_p;
                    x_p = x_p + x_p_dot * dt;

                    p_State_horizon(:, i) = x_p;
                end

                perturbed_cost = J([p_State_horizon; perturbed_controls], current_reference) + ...
                                 JN(p_State_horizon(:, N), current_reference(:, N));

                gradient(j, n) = (perturbed_cost - original_cost) / epsilon;
            end
        end

        % ---- Gradient descent update with step decay ----
        alpha_t = alpha / (1 + decay_rate * k);
        att_controls(:, 1:N) = att_controls(:, 1:N) - alpha_t * gradient;

    end

    % Return current cost for logging
    J_current = J([State_horizon; att_controls], current_reference) + ...
                JN(State_horizon(:, N), current_reference(:, N));
end
function [pos_controls,J_current]=a_Matt_MPC_Position_Controller(current_reference,X,pos_controls,N,J,JN,dt,K,angular_time_constants,decay_rate,alpha,epsilon)

%% Gradient Descent
    % Call Reference for this time horizon
    % reference is of form [positions;velocities;angles=0;thrust;last_angle]
    % angle has two punishments to ensure that angle change is not drastic and angles stay close to zero
 
    yaw=X(9);
    pos_controls(:,1:N-1)=pos_controls(:,2:N);
    for k= 1:K
        State_vector=X(1:8,1); % Current State Vector
        for i=1:N
            predicted_acceleration=acceleration_function([pos_controls(:,i);yaw]);
            State_vector_derivative=[State_vector(4);State_vector(5);State_vector(6);predicted_acceleration;(pos_controls(2,i)-State_vector(7))/angular_time_constants;(pos_controls(3,i)-State_vector(8))/angular_time_constants];
            State_vector=State_vector+State_vector_derivative*dt; 
            State_horizon(:,i)=State_vector;
        end
        current_reference(10:11,:)=[X(7:8),State_horizon(7:8,1:N-1)];

        original_cost=J([State_horizon;pos_controls],current_reference)+JN(State_horizon(1:8,N),current_reference(1:8,N));
        for n=1:N
            for j=1:3
                perturbed_pos_controls=pos_controls;
                perturbed_pos_controls(j,n)=perturbed_pos_controls(j,n)+epsilon;
                if n>1
                    p_State_vector=State_horizon(:,n-1);
                else
                    p_State_vector=X(1:8,1);
                end
                p_State_horizon(:,1:n)=State_horizon(:,1:n);
                for i=n:N
                    predicted_acceleration=acceleration_function([perturbed_pos_controls(:,i);yaw]);
                    p_State_vector_derivative=[p_State_vector(4);p_State_vector(5);p_State_vector(6);predicted_acceleration;(perturbed_pos_controls(2,i)-p_State_vector(7))/angular_time_constants;(perturbed_pos_controls(3,i)-p_State_vector(8))/angular_time_constants];
                    p_State_vector=p_State_vector+p_State_vector_derivative*dt;
                    p_State_horizon(1:8,i)=p_State_vector;
                end

                current_reference(10:11,:)=[X(7:8),p_State_horizon(7:8,1:N-1)];
                perturbed_cost=J([p_State_horizon;perturbed_pos_controls],current_reference)+JN(p_State_horizon(1:8,N),current_reference(1:8,N));
                gradient(j,n)=(perturbed_cost-original_cost)/epsilon;
            end

        end
       
        alpha_t = alpha / (1 + decay_rate * k);

        % Gradient update
        pos_controls(1:3,1:N) = pos_controls(1:3,1:N) - alpha_t * gradient;

    end

    J_current=J([State_horizon;pos_controls],current_reference)+JN(State_horizon(1:8,N),current_reference(1:8,N));
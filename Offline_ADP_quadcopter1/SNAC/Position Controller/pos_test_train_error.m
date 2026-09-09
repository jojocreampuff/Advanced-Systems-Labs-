function [mae_train, mse_train, mae_test, mse_test] = pos_test_train_error(x_k_train, Position_W, Position_R, Position_G, Position_F, Position_Q, A, N_states, N_patterns, X_min, X_max, Y_min, Y_max, Z_min, Z_max, u_min, u_max, v_min, v_max, w_min, w_max)

    %% training error 
    lambda_k_1_target_train = zeros(N_states, N_patterns);
    lambda_k_1_train = zeros(N_states, N_patterns);
    for i = 1 : length(x_k_train(1,:))
        x_k = x_k_train(:,i);

        % Running states through nerual network
        lambda_k_1_train(:,i) = Position_W' * Basis_Func_pos_NV(x_k); %step 1

        % Optimal control equation
        u_k_train = -Position_R^-1 * Position_G(x_k).' * lambda_k_1_train(:,i); %step 2

        % Discretized dynamics
        x_k_1_train = Position_F(x_k) + Position_G(x_k) * u_k_train; % step 3

        % States through nerual network
        lambda_k_2_train = Position_W' * Basis_Func_pos_NV(x_k_1_train); % step 4

        % Target costate equation
        A_k_1_train = A(x_k_1_train);
        lambda_k_1_target_train(:,i) = Position_Q * (x_k_1_train) + A_k_1_train.' * lambda_k_2_train;
    end
    
    % mean absolute error
    mae_train = mae(lambda_k_1_target_train - lambda_k_1_train);
    
    % mean squared error
    mse_train = mean(mean((lambda_k_1_target_train(:) - lambda_k_1_train(:)).^2));
    

    %% test error
    lambda_k_1_target_test = zeros(N_states, N_patterns);
    lambda_k_1_test = zeros(N_states, N_patterns);
    for i = 1 : length(x_k_train(1,:))
        X1 = X_min + (X_max - X_min) * rand;%(1, N_patterns);
        X2 = Y_min + (Y_max - Y_min) * rand;%(1, N_patterns);
        X3 = Z_min + (Z_max - Z_min) * rand;%(1, N_patterns);    
        X4 = u_min + (u_max - u_min) * rand;%(1, N_patterns);
        X5 = v_min + (v_max - v_min) * rand;%(1, N_patterns);
        X6 = w_min + (w_max - w_min) * rand;%(1, N_patterns);

        x_k_test = [X1; X2; X3; X4; X5; X6];
        % Running states through nerual network
        lambda_k_1_test(:,i) = Position_W' * Basis_Func_pos_NV(x_k_test); %step 1

        % Optimal control equation
        u_k_test = -Position_R^-1 * Position_G(x_k_test).' * lambda_k_1_test(:,i); %step 2

        % Discretized dynamics
        x_k_1_test = Position_F(x_k_test) + Position_G(x_k_test) * u_k_test; % step 3

        % States through nerual network
        lambda_k_2_test = Position_W' * Basis_Func_pos_NV(x_k_1_test); % step 4

        % Target costate equation
        A_k_1_test = A(x_k_1_test);
        lambda_k_1_target_test(:,i) = Position_Q * (x_k_1_test) + A_k_1_test.' * lambda_k_2_test;
    end
    
    
    % mean absolute error
    mae_test = mae(lambda_k_1_target_test - lambda_k_1_test);
    
    % mean squared error
    mse_test = mean(mean((lambda_k_1_target_test(:) - lambda_k_1_test(:)).^2));

end

function out = Basis_Func_pos_NV(x)
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);
x5 = x(5);
x6 = x(6);

out = [ 
    1
    x1
    x2
    x3
    x4
    x5
    x6
    x1^2
    x2^2
    x3^2
    x4^2
    x5^2
    x6^2
 ];
end
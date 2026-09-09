function [mae_train, mse_train, mae_test, mse_test] = att_test_train_error(x_k_train, Attitude_W, Attitude_R, Attitude_G, Attitude_F, Attitude_Q, a_col1, a_col2, a_col3, a_col4, a_col5, a_col6, PHI_min, PHI_max, THE_min, THE_max, PSI_min, PSI_max, p_min, p_max, q_min, q_max, r_min, r_max, N_patterns)

    %% training error
    lambda_k_1_train = Attitude_W' * Basis_Func_84(x_k_train);
    
    u_k_train = -Attitude_R^-1 * Attitude_G(x_k_train)' * lambda_k_1_train;
    
    x_k_1_train = Attitude_F(x_k_train) + Attitude_G(x_k_train)* u_k_train;
    
    lambda_k_2_train = Attitude_W' * Basis_Func_84(x_k_1_train);
    
    % Target costate equation
    A_col1 = a_col1(x_k_1_train);
    A_col2 = a_col2(x_k_1_train);
    A_col3 = a_col3(x_k_1_train);
    A_col4 = a_col4(x_k_1_train);
    A_col5 = a_col5(x_k_1_train);
    A_col6 = a_col6(x_k_1_train);
    
    L = lambda_k_2_train; % for simplicity
    
       KEEP_DIMS_train = [
    sum( A_col1(:,:).*L(:,:)); 
    sum( A_col2(:,:).*L(:,:));
    sum( A_col3(:,:).*L(:,:));
    sum( A_col4(:,:).*L(:,:));
    sum( A_col5(:,:).*L(:,:));
    sum( A_col6(:,:).*L(:,:));
    ]; % keep_dims must be 6x # train
    
    lambda_k_1_target_train = Attitude_Q * (x_k_1_train) + KEEP_DIMS_train;
    
    % mean absolute error
    mae_train = mae(lambda_k_1_target_train - lambda_k_1_train);
    
    % mean squared error
    mse_train = mean(mean((lambda_k_1_target_train(:) - lambda_k_1_train(:)).^2));
    

    X1 = PHI_min + (PHI_max - PHI_min) * rand(1, N_patterns);
    X2 = THE_min + (THE_max - THE_min) * rand(1, N_patterns);
    X3 = PSI_min + (PSI_max - PSI_min) *rand(1, N_patterns);    
    X4 = p_min + (p_max - p_min) * rand(1, N_patterns);
    X5 = q_min + (q_max - q_min) * rand(1, N_patterns);
    X6 = r_min + (r_max - r_min) * rand(1, N_patterns); 
    
    x_k_test = [X1; X2; X3; X4; X5; X6];
    
    lambda_k_1_test = Attitude_W' * Basis_Func_84(x_k_test);
    
    u_k_test = -Attitude_R^-1 * Attitude_G(x_k_test)' * lambda_k_1_test;
    
    x_k_1_test = Attitude_F(x_k_test) + Attitude_G(x_k_test)* u_k_test;
    
    lambda_k_2_test = Attitude_W' * Basis_Func_84(x_k_1_test);
    
    % Target costate equation
    A_col1 = a_col1(x_k_1_test);
    A_col2 = a_col2(x_k_1_test);
    A_col3 = a_col3(x_k_1_test);
    A_col4 = a_col4(x_k_1_test);
    A_col5 = a_col5(x_k_1_test);
    A_col6 = a_col6(x_k_1_test);
    
    L = lambda_k_2_test; % for simplicity
    
       KEEP_DIMS_test = [
    sum( A_col1(:,:).*L(:,:)); 
    sum( A_col2(:,:).*L(:,:));
    sum( A_col3(:,:).*L(:,:));
    sum( A_col4(:,:).*L(:,:));
    sum( A_col5(:,:).*L(:,:));
    sum( A_col6(:,:).*L(:,:));
    ]; % keep_dims must be 6x # train
    
    lambda_k_1_target_test = Attitude_Q * (x_k_1_test) + KEEP_DIMS_test;
    
    % mean absolute error
    mae_test = mae(lambda_k_1_target_test - lambda_k_1_test);
    
    % mean squared error
    mse_test = mean(mean((lambda_k_1_target_test(:) - lambda_k_1_test(:)).^2));

end
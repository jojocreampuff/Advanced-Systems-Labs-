function plot_training(t_full, err_full, theta_hist_full,P_norm_full, figTitle, step)
    %% use the step to reduce the plot file size (1.5 million points makes a > 1Gb figure size)
    t = t_full(1:step:end);
    err = err_full(:,1:step:end);
    theta_hist = theta_hist_full(:,1:step:end);
    P_norm = P_norm_full(1:step:end);
    figure;
    subplot(3,1,1);
    plot(t, err); grid on;
    title(figTitle ,"Interpreter","latex");
    xlabel("Time (s)","Interpreter","latex") 
    % ylim([-1 1])
    ylabel("State Error","Interpreter","latex");
    legend("$e_1$", "$e_2$", "$e_3$", "$e_4$","$e_5$", "$e_6$", "Interpreter","latex");

    subplot(3,1,2);
    plot(t, theta_hist); grid on;
    xlabel("Time (s)","Interpreter","latex"); ylabel("$\hat{\theta}$","Interpreter","latex");
    title("Weight evolution","Interpreter","latex");
    subplot(3,1,3);
    plot(t, P_norm); grid on;
    xlabel("Time (s)","Interpreter","latex"); ylabel("$||P||$","Interpreter","latex");
    title("Covariance Norm","Interpreter","latex");
end

% figure;
% subplot(2,1,1);
% plot(trainUnsafe.t, trainUnsafe.err); grid on;
% xlabel("Time (s)"); ylabel("e = x - r");
% title("Error evolution: Unsafe Training")
% subplot(2,1,2);
% plot(trainUnsafe.t, trainUnsafe.theta_hist); grid on;
% xlabel("Time (s)"); ylabel("\theta components");
% title("Weight evolution: Unsafe Training");
% 
% figure;
% subplot(2,1,1);
% plot(trainSafe.t, trainSafe.err); grid on;
% xlabel("Time (s)"); ylabel("e = x - r");
% title("Error evolution during safe training")
% subplot(2,1,2);
% plot(trainSafe.t, trainSafe.theta_hist); grid on;
% xlabel("Time (s)"); ylabel("\theta components");
% title("Weight evolution");
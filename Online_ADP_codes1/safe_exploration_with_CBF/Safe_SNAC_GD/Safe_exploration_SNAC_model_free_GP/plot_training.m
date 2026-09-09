function plot_training(t_full, err_full, theta_hist_full, figTitle, step)
    %% use the step to reduce the plot file size (1.5 million points makes a > 1Gb figure size)
    t = t_full(1:step:end);
    err = err_full(:,1:step:end);
    theta_hist = theta_hist_full(:,1:step:end);
    
    % figure;
    % subplot(2,1,1);
    % hold on; grid on; box on;
    % set(gca, 'GridLineStyle', ':');
    % plot(t, err);
    % xlabel("Time (s)","Interpreter","latex") 
    % ylabel("State Error","Interpreter","latex");
    % legend("$e_1$", "$e_2$", "Interpreter","latex");
    % subplot(2,1,2);
    % hold on; grid on; box on;
    % set(gca, 'GridLineStyle', ':');
    % plot(t, theta_hist);
    % xlabel("Time (s)","Interpreter","latex"); ylabel("$\hat{\theta}$","Interpreter","latex");
    % title(figTitle,"Interpreter","latex");

    figure;
    subplot(2,1,1)
    hold on; grid on; box on;
    set(gca, 'GridLineStyle', ':');
    plot(t, err(1,:));
    xlabel("Time (s)","Interpreter","latex") 
    ylabel("$e_1$","Interpreter","latex");

    subplot(2,1,2)
    hold on; grid on; box on;
    set(gca, 'GridLineStyle', ':');
    plot(t, err(2,:));
    xlabel("Time (s)","Interpreter","latex") 
    ylabel("$e_2$","Interpreter","latex");


    figure;
    hold on; grid on; box on;
    set(gca, 'GridLineStyle', ':');
    plot(t, theta_hist);
    xlabel("Time (s)","Interpreter","latex"); ylabel("$\hat{\theta}$","Interpreter","latex");
    % title("Weight evolution","Interpreter","latex");
end

    % %% use the step to reduce the plot file size (1.5 million points makes a > 1Gb figure size)
    % t = t_full(1:step:end);
    % err = err_full(:,1:step:end);
    % theta_hist = theta_hist_full(:,1:step:end);
    % figure;
    % subplot(2,1,1);
    % plot(t, err); grid on;
    % title(figTitle ,"Interpreter","latex");
    % xlabel("Time (s)","Interpreter","latex") 
    % ylabel("State Error","Interpreter","latex");
    % legend("$e_1$", "$e_2$", "Interpreter","latex");
    % 
    % subplot(2,1,2);
    % plot(t, theta_hist); grid on;
    % xlabel("Time (s)","Interpreter","latex"); ylabel("$\hat{\theta}$","Interpreter","latex");
    % title("Weight evolution","Interpreter","latex");

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
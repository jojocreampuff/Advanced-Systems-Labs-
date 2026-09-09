function plot_GP_drift_stuff(trainOut, figTitle, step)
%% use the step to reduce the plot file size (1.5 million points makes a > 1Gb figure size)
    t = trainOut.t(:)';
    t = t(1:step:end);
    Hhat_hist = trainOut.Hhat_hist';
    Hhat_hist = Hhat_hist(:,1:step:end);
    E_norm = trainOut.E_norm(:)';
    E_norm = E_norm(:,1:step:end);
    t_gp = trainOut.gp_update_times(:)';
    t_gp = t_gp(:,1:step:end);

    figure('Name', figTitle);

    subplot(3,1,1);
    plot(t, Hhat_hist);
    grid on;
    ylabel("$\hat{H}$", "Interpreter","latex");
    title(figTitle, "Interpreter","latex");

    subplot(3,1,2);
    plot(t, E_norm);
    grid on;
    ylabel("$||x-\hat{x}||$", "Interpreter","latex");

    subplot(3,1,3);
    if isempty(t_gp)
        plot(t, zeros(size(t)), 'LineWidth', 1.0);
        ylim([-0.1 1.1]);
    else
        stem(t_gp, ones(size(t_gp)));
        ylim([-0.1 1.1]);
    end
    grid on;
    xlabel("Time (s)", "Interpreter","latex");
    ylabel("GP upd", "Interpreter","latex");
end
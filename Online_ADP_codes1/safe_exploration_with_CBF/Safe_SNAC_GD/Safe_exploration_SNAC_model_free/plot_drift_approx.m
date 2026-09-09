function plot_drift_approx(trainOut, figTitle, step)
%% use the step to reduce the plot file size (1.5 million points makes a > 1Gb figure size)
    t = trainOut.t(:)';
    t = t(1:step:end);
    Hhat_hist = trainOut.Hhat_hist';
    Hhat_hist = Hhat_hist(:,1:step:end);
    E_norm = trainOut.E_norm(:)';
    E_norm = E_norm(:,1:step:end);

    figure('Name', figTitle);

    subplot(2,1,1);
    plot(t, Hhat_hist);
    grid on;
    ylabel("$\hat{H}$", "Interpreter","latex");
    title(figTitle, "Interpreter","latex");

    subplot(2,1,2);
    plot(t, E_norm);
    grid on;
    ylabel("$||x-\hat{x}||$", "Interpreter","latex");

end
function metrics_table = extract_mc_multiagent_metrics(mc_results, dt)

controllers = fieldnames(mc_results);
nC = numel(controllers);

stats = struct( ...
    'Controller', cell(nC,1), ...
    'ControlCost_mean', [], ...
    'ControlCost_std', [], ...
    'Tracking_mean', [], ...
    'Tracking_std', [], ...
    'LeaderTracking_mean', [], ...
    'LeaderTracking_std', [], ...
    'WorstTracking_mean', [], ...
    'WorstTracking_std', [], ...
    'Formation_mean', [], ...
    'Formation_std', [], ...
    'SimTime_mean', [], ...
    'SimTime_std', [] );

for k = 1:nC
    name = controllers{k};
    runs = mc_results.(name).run;
    nRuns = numel(runs);

    control_cost = zeros(1,nRuns);
    pos_rmse = zeros(1,nRuns);
    leader_rmse = zeros(1,nRuns);
    worst_rmse = zeros(1,nRuns);
    form_rmse = zeros(1,nRuns);
    simtime = zeros(1,nRuns);

    for r = 1:nRuns
        res = runs(r);

        X = res.X;                 % 12 x N x Nd
        X_ref = res.X_ref;         % 12 x N x Nd
        U = res.U;                 % 4 x N x Nd
        F = res.formation_error;   % 3 x N x Nd

                % skip diverged runs
        if any(~isfinite(X(:))) || any(~isfinite(X_ref(:))) || any(~isfinite(U(:))) || any(~isfinite(F(:)))
            continue
        end

        Nd = size(X,3);

        % control cost
        cost = 0;
        for d = 1:Nd
            Ud = U(:,:,d);
            cost = cost + dt * sum(Ud(:).^2);
        end
        control_cost(r) = cost;

        % tracking
        pos_rmse_agents = zeros(1,Nd);
        for d = 1:Nd
            e = X(1:3,:,d) - X_ref(1:3,:,d);
            pos_rmse_agents(d) = sqrt(mean(sum(e.^2,1)));
        end

        pos_rmse(r) = sqrt(mean(pos_rmse_agents.^2));
        leader_rmse(r) = pos_rmse_agents(1);
        worst_rmse(r) = max(pos_rmse_agents);

        % formation
        form_agents = zeros(1,Nd-1);
        for d = 2:Nd
            ferr = F(:,:,d);
            form_agents(d-1) = sqrt(mean(sum(ferr.^2,1)));
        end
        form_rmse(r) = sqrt(mean(form_agents.^2));

        % sim time
        simtime(r) = res.simtime;
    end

    stats(k).Controller = name;
    stats(k).ControlCost_mean = mean(control_cost);
    stats(k).ControlCost_std  = std(control_cost);

    stats(k).Tracking_mean = mean(pos_rmse);
    stats(k).Tracking_std  = std(pos_rmse);

    stats(k).LeaderTracking_mean = mean(leader_rmse);
    stats(k).LeaderTracking_std  = std(leader_rmse);

    stats(k).WorstTracking_mean = mean(worst_rmse);
    stats(k).WorstTracking_std  = std(worst_rmse);

    stats(k).Formation_mean = mean(form_rmse);
    stats(k).Formation_std  = std(form_rmse);

    stats(k).SimTime_mean = mean(simtime);
    stats(k).SimTime_std  = std(simtime);
end

metrics_table = table( ...
    string({stats.Controller}).', ...
    [stats.ControlCost_mean].', ...
    [stats.ControlCost_std].', ...
    [stats.Tracking_mean].', ...
    [stats.Tracking_std].', ...
    [stats.Formation_mean].', ...
    [stats.Formation_std].', ...
    [stats.SimTime_mean].', ...
    'VariableNames', { ...
        'Controller', ...
        '\makecell{Mean Control\\ Cost}', ...
        '\makecell{Std Control\\ Cost}', ...
        '\makecell{Mean RMS \\Tracking Error}', ...
        '\makecell{Std RMS \\Tracking Error}', ...
        '\makecell{Mean RMS \\Formation Error}', ...
        '\makecell{Std RMS \\Formation Error}', ...
        '\makecell{Mean\\ SimTime}' ...
    } ...
);

end
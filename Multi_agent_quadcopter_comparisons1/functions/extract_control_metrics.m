function [metrics, controller_names, metrics_table] = extract_control_metrics(global_results, Ts_Sim)
    controller_names = fieldnames(global_results);
    nC = numel(controller_names);

    % Preallocate
    metrics.control_cost        = nan(1, nC);
    metrics.control_mags        = nan(4, nC);
    metrics.control_smoothness  = nan(1, nC);
    metrics.att_rmse = nan(1, nC);
    metrics.pos_rmse    = nan(1, nC);
    metrics.simtime             = nan(1, nC);

    for k = 1:nC
        name = controller_names{k};
        res = global_results.(name);

        % -------------------- Controls --------------------
        if ~isfield(res, 'U') || isempty(res.U)
            warning('Controller "%s" missing U. Skipping control metrics.', name);
        else
            U = res.U;

            % Ensure U is 4xN (if Nx4, transpose)
            if size(U,1) ~= 4 && size(U,2) == 4
                U = U.';
            end

            if size(U,1) == 4
                % Control cost: prefer stored (all_cost = [tot, track, control])
                if isfield(res, 'all_cost') && ~isempty(res.all_cost) && numel(res.all_cost) >= 3
                    metrics.control_cost(k) = res.all_cost(3);
                else
                    % Fallback: integral u'u dt
                    metrics.control_cost(k) = Ts_Sim * sum(sum(U.^2, 1));
                end

                % Max magnitude per channel
                metrics.control_mags(:,k) = max(abs(U), [], 2);

                % Smoothness: integral ||du/dt||^2 dt
                if size(U,2) >= 2
                    dU = diff(U, 1, 2) / Ts_Sim;
                    metrics.control_smoothness(k) = Ts_Sim * sum(sum(dU.^2, 1));
                end
            else
                warning('Controller "%s" has U size %dx%d (expected 4xN).', ...
                    name, size(U,1), size(U,2));
            end
        end

        % -------------------- Tracking error --------------------
        pos_error = res.X(1:3,:) - res.X_ref(1:3,:);
        metrics.pos_rmse(k) = sqrt(mean(sum(pos_error.^2, 1)));

        % -------------------- Simulation time --------------------
        if isfield(res, 'simtime') && ~isempty(res.simtime)
            metrics.simtime(k) = res.simtime;
        end
        
     % -------------------- attitude error --------------------
        % att_error = res.X(7:9,:) - res.X_ref(7:9,:);
        % metrics.att_rmse(k) = Ts_Sim * sum( sqrt(sum(att_error.^2, 1)) );
        att_error = res.error(7:9,:);
        metrics.att_rmse(k) = sqrt(mean(sum(att_error.^2, 1)));

    end

    % -------------------- Build metrics table --------------------
    metrics_table = table( ...
        controller_names, ...
        metrics.control_cost.', ...
        metrics.pos_rmse.', ...
        metrics.simtime.', ...
        'VariableNames', { ...
            'Controller', ...
            '\makecell{Control \\Cost}', ...
            '\makecell{RMS Tracking\\ Error (m)}', ...
            'SimTime'
        } ...
    );
end
    % 
    % % -------------------- Build metrics table --------------------
    % metrics_table = table( ...
    %     controller_names, ...
    %     metrics.control_cost.', ...
    %     metrics.control_smoothness.', ...
    %     metrics.pos_rmse.', ...
    %     metrics.att_rmse.', ...
    %     metrics.simtime.', ...
    %     'VariableNames', { ...
    %         'Controller', ...
    %         '\makecell{Control \\Cost}', ...
    %         '\makecell{Control \\Smoothness}', ...
    %         '\makecell{RMS Tracking\\ Error (m)}', ...
    %         '\makecell{RMS Attitude \\Error (rad)}', ...
    %         'SimTime'
    %     } ...
    % );
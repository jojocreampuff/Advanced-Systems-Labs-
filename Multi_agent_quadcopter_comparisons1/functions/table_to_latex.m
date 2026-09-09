function table_to_latex(T, filename, captionText, labelText)
% TABLE_TO_LATEX  Write a MATLAB table to a LaTeX table* environment.
%
% Usage:
%   table_to_latex(Tmetrics, 'control_metrics_table.tex', 'My caption', 'tab:metrics');
%
% Notes:
% - Writes a table* float with \centering, \caption{}, \label{}, and a tabular.
% - Keeps the same simple formatting as your original function.

    fid = fopen(filename, 'w');
    if fid < 0
        error('Could not open file: %s', filename);
    end

    % Defaults if caption/label omitted
    if nargin < 3 || isempty(captionText)
        captionText = 'xxx';
    end
    if nargin < 4 || isempty(labelText)
        labelText = 'tab:xxx';
    end

    % Begin table* environment
    fprintf(fid, '\\begin{table*}[h]\n');
    fprintf(fid, '\\centering\n');

    % Tabular
    fprintf(fid, '\\begin{tabular}{%s}\n', repmat('c', 1, width(T)));
    fprintf(fid, '\\hline\n');

    % Header row
    headers = T.Properties.VariableNames;
    for i = 1:length(headers)
        % Escape underscores in variable names for LaTeX
        h = strrep(headers{i}, '_', '\\_');
        fprintf(fid, '%s', h);
        if i < length(headers)
            fprintf(fid, ' & ');
        else
            fprintf(fid, ' \\\\ \n');
        end
    end
    fprintf(fid, '\\hline\n');

    % Data rows
    for i = 1:height(T)
        for j = 1:width(T)
            val = T{i,j};

            if isnumeric(val)
                if isscalar(val)
                    fprintf(fid, '%.4f', val);
                else
                    % If a cell contains a numeric vector/matrix, print compactly
                    fprintf(fid, '%s', mat2str(val));
                end
            else
                s = string(val);
                % Escape underscores in string data too
                s = strrep(s, "_", "\_");
                fprintf(fid, '%s', s);
            end

            if j < width(T)
                fprintf(fid, ' & ');
            else
                fprintf(fid, ' \\\\ \n');
            end
        end
    end

    fprintf(fid, '\\hline\n');
    fprintf(fid, '\\end{tabular}\n');

    % Caption + label + end environment
    fprintf(fid, '\\caption{%s}\n', captionText);
    fprintf(fid, '\\label{%s}\n', labelText);
    fprintf(fid, '\\end{table*}\n');

    fclose(fid);
end
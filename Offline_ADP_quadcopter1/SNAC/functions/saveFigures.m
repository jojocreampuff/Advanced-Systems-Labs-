function saveFigures(filename)
    if ~exist("figures", 'dir')
      mkdir("figures")
    end
    filepath = strcat('figures/', filename, '/');
    
    figList = findobj('type','figure');
    
    mkdir(filepath)
    addpath(genpath(filepath))

  for i = 1:length(figList)
      h = figList(i);

      filename = strcat(filepath, 'figure_', num2str(i));      
      
      savefig(h, filename)
      set(h, 'Units', 'Inches');
      pos = get(h, 'Position');
      set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos(3), pos(4)])
      print(h, filename, '-dpdf', '-r600')
      print(h, filename, '-dtiff', '-r600')
  end

  disp("Figures saved!")

%   evalin('base', strcat('save("', filepath, 'workspace.mat")'));
%   disp("Workspace saved")
end
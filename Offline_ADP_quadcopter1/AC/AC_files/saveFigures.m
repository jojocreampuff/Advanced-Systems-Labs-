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
      
      set(h, 'Units', 'Inches');
      pos = get(h, 'Position');
      set(h, 'PaperPositionMode', 'Auto', 'PaperUnits', 'Inches', 'PaperSize', [pos(3), pos(4)])

      savefig(h, filename)

      print(h, filename, '-dpdf', '-r0')

      print(h, filename, '-dtiff', '-r0')
  end

  disp("Figures saved!")


  evalin('base', strcat('save("', filepath, 'workspace.mat")'));
  disp("Workspace saved!")
end
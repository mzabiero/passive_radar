function terminal = log_workspace_panel(fig,position)
    terminal.pLog = uipanel(fig,"Title","Log","Position",position);
    
    terminal.LogCategories = {'General', 'CAF Power', 'Processing', 'Parameters'};
    terminal.LogData = containers.Map(terminal.LogCategories, repmat({''}, size(terminal.LogCategories)));
    
    % Create a horizontal tree (categories)
    terminal.LogTree = uitree(terminal.pLog, ...
        'Position',position, ...
        'Orientation','horizontal');

end
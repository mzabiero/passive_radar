function ax = plotCafVelCut(cafMap, rangeIdx, dopplerAxis, margin, ax, ttl)
    if nargin < 5 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Velocity Cuts', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    
    rStart = max(1, rangeIdx - margin);
    rStop = min(size(cafMap, 1), rangeIdx + margin);
    cafSlice = cafMap(rStart:rStop, :);
    % 
    % if nargin < 6 
    %     %tStr = sprintf('Velocity Cuts (Range Idx: %d \\pm %d)', rangeIdx, margin);
    %     tStr = sprintf(, rangeIdx, margin);
    % else
    %     tStr = sprintf('Przekrój prędkościowy (Indeks odległości: %d \\pm %d) %s', rangeIdx, margin, ttl);
    %     %tStr = sprintf('Velocity Cuts (Range Idx: %d \\pm %d) %s', rangeIdx, margin, ttl);
    % end
    
    lineObjs = findobj(ax, 'Type', 'line');
    if isempty(lineObjs)
        plot(ax, dopplerAxis, cafSlice');
        grid(ax, 'on');
        xlabel(ax, 'Prędkość bistatyczna [m/s]','FontSize',14);
        ylabel(ax, 'Moc CAF [dB]', 'FontSize',14);
        %title(ax, tStr);
    else
        delete(lineObjs);
        plot(ax, dopplerAxis, cafSlice');
        grid(ax, 'on');
        xlabel(ax, 'Prędkość bistatyczna [m/s]','FontSize',14);
        ylabel(ax, 'Moc CAF [dB]','FontSize',14);
        %ax.Title.String = tStr;
    end
end
function ax = plotCorr(lags, corrVec, ax)
    if nargin < 3 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Cross-Correlation', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    
    lineObj = findobj(ax, 'Type', 'line');
    if isempty(lineObj)
        plot(ax, lags, corrVec);
        grid(ax, 'on');
        xlabel(ax, 'Opóźnienie [próbki]');
        ylabel(ax, 'Moduł Korelacji');
        title(ax, 'Funkcja Korelacji');
    else
        lineObj.XData = lags;
        lineObj.YData = corrVec;
    end
end
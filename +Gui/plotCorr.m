function ax = plotCorr(lags, corrVec, ax)
    if nargin < 3 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Cross-Correlation', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    cla(ax);
    plot(ax, lags, abs(corrVec));
    grid(ax, 'on');
    
    xlabel(ax, 'Opóźnienie [próbki]');
    ylabel(ax, 'Moduł Korelacji');
    title(ax, 'Funkcja Korelacji');
end
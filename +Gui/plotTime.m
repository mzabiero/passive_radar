function ax = plotTime(sig, fs, ax, ttl)
    if nargin < 3 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Time Domain', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    
    N = length(sig);
    timeAxis = (0:N-1) / fs;
    
    if nargin < 4
        tStr = 'Przebieg czasowy sygnału';
    else
        tStr = ttl;
    end
    
    lineObj = findobj(ax, 'Type', 'line');
    if isempty(lineObj)
        plot(ax, timeAxis, real(sig));
        grid(ax, 'on');
        xlabel(ax, 'Czas [s]','FontSize',14);
        ylabel(ax, 'Amplituda','FontSize',14);
        title(ax, tStr);
    else
        lineObj.XData = timeAxis;
        lineObj.YData = real(sig);
        ax.Title.String = tStr;
    end
end
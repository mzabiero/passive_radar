function ax = plotTime(sig, fs, ax, ttl)
    if nargin < 3 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Time Domain', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    cla(ax);
    N = length(sig);
    timeAxis = (0:N-1) / fs;
    
    plot(ax, timeAxis, real(sig));
    grid(ax, 'on');
    
    xlabel(ax, 'Czas [s]');
    ylabel(ax, 'Amplituda');
    if nargin < 4
        title(ax, 'Przebieg czasowy sygnału');
    else
        title(ax, ttl);
    end
end
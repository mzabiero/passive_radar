function ax = plotSTFT(stftMap, timeAxis, dopplerAxis, ax, ttl)
    if nargin < 4 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'STFT Micro-Doppler', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    
    imgObj = findobj(ax, 'Type', 'image');
    if isempty(imgObj)
        imagesc(ax, timeAxis, dopplerAxis, stftMap);
        ax.YDir = 'normal';
        colormap(ax, 'jet');
        colorbar(ax);
        axis(ax, 'xy');
        xlabel(ax, 'Czas [s]');
        ylabel(ax, 'Doppler [Hz]');
        if nargin < 5
            title(ax, 'STFT (Micro-Doppler)');
        else
            title(ax, ttl);
        end
    else
        imgObj.CData = stftMap;
        imgObj.XData = timeAxis;
        imgObj.YData = dopplerAxis;
        if nargin >= 5
            ax.Title.String = ttl;
        end
    end
    
    meanVal = mean(stftMap(:), "all");
    maxVal  = max(stftMap(:));  
    if ~isnan(maxVal) && ~isnan(meanVal) && (maxVal > meanVal)
        clim(ax, [meanVal maxVal]);
    end
end
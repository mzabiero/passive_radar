function ax = plotWVD(tfMap, timeAxis, dopplerAxis, ax, ttl)
    if nargin < 4 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'SPWVD Micro-Doppler', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    
    imgObj = findobj(ax, 'Type', 'image');
    if isempty(imgObj)
        imagesc(ax, timeAxis, dopplerAxis, tfMap);
        ax.YDir = 'normal';
        colormap(ax, 'jet');
        colorbar(ax);
        axis(ax, 'xy');
        xlabel(ax, 'Czas [s]');
        ylabel(ax, 'Doppler [Hz]');
        
        if nargin < 5
            title(ax, 'SPWVD (Micro-Doppler)');
        else
            title(ax, ttl);
        end
    else
        imgObj.CData = tfMap;
        imgObj.XData = timeAxis;
        imgObj.YData = dopplerAxis;
        
        if nargin >= 5
            ax.Title.String = ttl;
        end
    end
    
    meanVal = mean(tfMap(:), "all");
    maxVal  = max(tfMap(:));  
    
    if ~isnan(maxVal) && ~isnan(meanVal) && (maxVal > meanVal)
        clim(ax, [meanVal maxVal]);
    end
end
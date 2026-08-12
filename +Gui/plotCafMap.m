function ax = plotCafMap(cafMap, rangeAxis, dopplerAxis, ax, plotMax)
    if nargin < 4 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'CAF Map (Local)', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    if nargin < 5
        plotMax = false;
    end
    
    imagesc(ax, dopplerAxis, rangeAxis, cafMap);
    ax.YDir = 'normal';
    
    colormap(ax, 'jet');
    colorbar(ax);
    axis(ax, 'xy');
    meanCaf = mean(cafMap(:),"all");
    maxCaf  = max(cafMap(:));  
    clim(ax,[meanCaf maxCaf]);
    
    xlabel(ax, 'Doppler [Hz]');
    ylabel(ax, 'Range [km]');
    title(ax, 'Cross Ambiguity Function');
    if plotMax    
        [~, maxIdx] = max(cafMap, [], 'all');
        [rIdx, dIdx] = ind2sub(size(cafMap), maxIdx);
        
        peakRange = rangeAxis(rIdx);
        peakDoppler = dopplerAxis(dIdx);
        
        hold(ax, 'on');
        plot(ax, peakDoppler, peakRange, 'r+', 'MarkerSize', 12, 'LineWidth', 2);
        hold(ax, 'off');
    end
end
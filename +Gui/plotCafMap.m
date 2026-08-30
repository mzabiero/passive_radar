function ax = plotCafMap(cafMap, rangeAxis, dopplerAxis, ax, plotMax)
    if nargin < 4 || isempty(ax) || ~isvalid(ax)
        %fig = figure('Name', 'CAF Map (Local)', 'NumberTitle', 'off');
        fig = figure;
        ax = axes(fig);
    end
    if nargin < 5
        plotMax = false;
    end
    
    imgObj = findobj(ax, 'Type', 'image');
    if isempty(imgObj)
        imagesc(ax, dopplerAxis, rangeAxis, cafMap);
        ax.YDir = 'normal';
        colormap(ax, 'jet');
        colorbar(ax);
        axis(ax, 'xy');
        xlabel(ax, 'Prędkość bistatyczna [m/s]','FontSize',14);
        ylabel(ax, 'Odegłość bistatyczna [km]','FontSize',14);
        zlabel(ax, 'Amplituda [dB]','FontSize',14);
        %title(ax, 'Cross Ambiguity Function');
    else
        imgObj.CData = cafMap;
        imgObj.XData = dopplerAxis;
        imgObj.YData = rangeAxis;
    end
    
    meanCaf = mean(cafMap(:), "all");
    maxCaf  = max(cafMap(:));  
    clim(ax, [meanCaf maxCaf]);
    axis(ax, 'tight');
    markerObj = findobj(ax, 'Tag', 'MaxPeakMarker');
    if plotMax    
        [~, maxIdx] = max(cafMap, [], 'all');
        [rIdx, dIdx] = ind2sub(size(cafMap), maxIdx);
        peakRange = rangeAxis(rIdx);
        peakDoppler = dopplerAxis(dIdx);
        
        if isempty(markerObj)
            hold(ax, 'on');
            plot(ax, peakDoppler, peakRange, 'r+', 'MarkerSize', 12, 'LineWidth', 2, 'Tag', 'MaxPeakMarker');
            hold(ax, 'off');
        else
            markerObj.XData = peakDoppler;
            markerObj.YData = peakRange;
        end
    else
        if ~isempty(markerObj)
            delete(markerObj);
        end
    end
end
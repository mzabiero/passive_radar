function ax = plotCafVelCut(cafMap, rangeIdx, dopplerAxis, margin, ax, ttl)
    if nargin < 5 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Velocity Cuts', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    
    
    rStart = max(1, rangeIdx - margin);
    rStop = min(size(cafMap, 1), rangeIdx + margin);
    
    cafSlice = cafMap(rStart:rStop, :);
    
    plot(ax, dopplerAxis, cafSlice');
    grid(ax, 'on');
    
    xlabel(ax, 'Doppler [Hz]');
    ylabel(ax, 'Power CAF [dB]');
    if nargin < 6 
        title(ax, sprintf('Velocity Cuts (Range Idx: %d \\pm %d)', rangeIdx, margin));
    else
        title(ax, sprintf('Velocity Cuts (Range Idx: %d \\pm %d) %s', rangeIdx, margin, ttl));
    end
end
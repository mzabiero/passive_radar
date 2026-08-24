function ax = plotInterpolate3Point(vals, delta, method, ax, ttl)
    if nargin < 4 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Interpolation View', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    
    if isempty(vals) || length(vals) ~= 3
        return;
    end
    
    xDiscrete = [-1, 0, 1];
    
    if method == "Interpolate"
        y1 = vals(1); 
        y2 = vals(2); 
        y3 = vals(3);
        a = (y1 - 2*y2 + y3) / 2;
        b = (y3 - y1) / 2;
        c = y2;
        
        xHighRes = linspace(-1.5, 1.5, 100);
        yHighRes = a * xHighRes.^2 + b * xHighRes + c;
        yPeak = a * delta^2 + b * delta + c;
    else
        xHighRes = xDiscrete;
        yHighRes = vals;
        yPeak = vals(2);
    end
    
    lineObj = findobj(ax, 'Tag', 'DiscretePoints');
    curveObj = findobj(ax, 'Tag', 'InterpCurve');
    peakObj = findobj(ax, 'Tag', 'PeakPoint');
    
    if isempty(lineObj)
        cla(ax);
        hold(ax, 'on');
        plot(ax, xDiscrete, vals, 'bo', 'MarkerSize', 6, 'LineWidth', 2, 'Tag', 'DiscretePoints');
        if method == "Interpolate"
            plot(ax, xHighRes, yHighRes, 'b--', 'Tag', 'InterpCurve');
        end
        plot(ax, delta, yPeak, 'r*', 'MarkerSize', 10, 'LineWidth', 2, 'Tag', 'PeakPoint');
        hold(ax, 'off');
        grid(ax, 'on');
        xlabel(ax, 'Przesunięcie względem dyskretnego maksimum [bin]');
        ylabel(ax, 'Amplituda');
        
        if nargin >= 5
            title(ax, ttl);
        else
            title(ax, 'Interpolacja paraboliczna 3-punktowa');
        end
    else
        lineObj.YData = vals;
        
        if method == "Interpolate"
            if isempty(curveObj)
                hold(ax, 'on');
                plot(ax, xHighRes, yHighRes, 'b--', 'Tag', 'InterpCurve');
                hold(ax, 'off');
            else
                curveObj.YData = yHighRes;
            end
        else
            if ~isempty(curveObj)
                delete(curveObj);
            end
        end
        
        peakObj.XData = delta;
        peakObj.YData = yPeak;
        
        if nargin >= 5
            ax.Title.String = ttl;
        end
    end
end
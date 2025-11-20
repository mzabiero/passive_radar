function bistatic_ellipse_gui()
% BISTATIC_ELLIPSE_GUI - Interactive tool to overlay bistatic ellipses and antenna sector
% Usage: run this file in MATLAB. It opens a UI where you can enter TX/RX coords,
% add bistatic ranges (m), rotate antenna pointing, change beamwidth, toggle layers,
% and save the current figure (.png and .mat).
%
% Notes / assumptions:
% - Inputs lat/lon are in degrees (WGS84). Altitudes are assumed 0 (ground).
% - Receiver (RX) is used as ENU origin.
% - Single transmitter (TX) and single receiver (RX).
% - Ellipse exists only if bistatic_range >= distance(TX,RX).
% - Sector model is simple: az_center +/- half_beamwidth (degrees clockwise from N).
% - Mapping Toolbox recommended (uses geoplot). If unavailable, code falls back to plain axes.

    close all;

    % Create app data store
    app = struct();
    app.ellipses = {}; % cell array of structs: {range_m, color, visible}
    app.colors = lines(10);
    app.nextColor = 1;

    % Default values
    app.tx = [50.06143, 19.93658]; % example: Krakow
    app.rx = [50.063, 19.94];
    app.az_center = 0; % degrees clockwise from North
    app.beamwidth = 20; % degrees full-width
    app.nEllipsePoints = 1000;

    % Create UI
    fig = uifigure('Name','Bistatic ellipse & antenna sector','Position',[200 200 1200 700]);
    app.fig = fig;

    % Left panel: controls
    ctrlPanel = uipanel(fig,'Title','Controls','Position',[10 10 300 680]);

    uilabel(ctrlPanel,'Text','TX Latitude:', 'Position',[10 620 100 20]);
    txLatEdit = uieditfield(ctrlPanel,'numeric','Value',app.tx(1),'Position',[120 620 160 22]);

    uilabel(ctrlPanel,'Text','TX Longitude:', 'Position',[10 590 100 20]);
    txLonEdit = uieditfield(ctrlPanel,'numeric','Value',app.tx(2),'Position',[120 590 160 22]);

    uilabel(ctrlPanel,'Text','RX Latitude:', 'Position',[10 560 100 20]);
    rxLatEdit = uieditfield(ctrlPanel,'numeric','Value',app.rx(1),'Position',[120 560 160 22]);

    uilabel(ctrlPanel,'Text','RX Longitude:', 'Position',[10 530 100 20]);
    rxLonEdit = uieditfield(ctrlPanel,'numeric','Value',app.rx(2),'Position',[120 530 160 22]);

    % Update positions button
    btnUpdatePos = uibutton(ctrlPanel,'push','Text','Update TX/RX','Position',[60 490 180 30],...
        'ButtonPushedFcn',@(btn,event) updatePos());

    % Ellipse input
    uilabel(ctrlPanel,'Text','Add bistatic range (m):','Position',[10 450 160 20]);
    rangeEdit = uieditfield(ctrlPanel,'numeric','Value',10000,'Position',[10 425 140 24]);

    btnAddEllipse = uibutton(ctrlPanel,'push','Text','Add Ellipse','Position',[160 425 120 24],...
        'ButtonPushedFcn',@(btn,event) addEllipse());

    % Listbox of ellipses
    uilabel(ctrlPanel,'Text','Ellipses (toggle):','Position',[10 380 160 20]);
    ellipseList = uilistbox(ctrlPanel,'Position',[10 240 280 140],'Items',{}, 'Multiselect','off',...
        'ValueChangedFcn',@(lb,ev) toggleEllipse());

    btnRemove = uibutton(ctrlPanel,'push','Text','Remove Selected','Position',[10 210 140 24],...
        'ButtonPushedFcn',@(btn,event) removeEllipse());

    btnClear = uibutton(ctrlPanel,'push','Text','Clear All','Position',[150 210 140 24],...
        'ButtonPushedFcn',@(btn,event) clearEllipses());

    % Antenna controls
    uilabel(ctrlPanel,'Text','Antenna azimuth (deg N->E):','Position',[10 170 200 20]);
    azSlider = uislider(ctrlPanel,'Position',[10 150 280 3],'Limits',[0 360],'Value',app.az_center,...
        'ValueChangedFcn',@(s,e) azChange(s.Value));
    azValLabel = uilabel(ctrlPanel,'Text',sprintf('Az = %.1f°',app.az_center),'Position',[10 130 200 20]);

    uilabel(ctrlPanel,'Text','Beamwidth (deg):','Position',[10 100 200 20]);
    bwSlider = uislider(ctrlPanel,'Position',[10 80 280 3],'Limits',[1 180],'Value',app.beamwidth,...
        'ValueChangedFcn',@(s,e) bwChange(s.Value));
    bwValLabel = uilabel(ctrlPanel,'Text',sprintf('BW = %.1f°',app.beamwidth),'Position',[10 60 200 20]);

    % Layer toggles
    chkShowEllipses = uicheckbox(ctrlPanel,'Text','Show Ellipses','Position',[10 30 150 20],'Value',true,...
        'ValueChangedFcn',@(c,e) refreshPlot());
    chkShowSector = uicheckbox(ctrlPanel,'Text','Show Sector','Position',[160 30 130 20],'Value',true,...
        'ValueChangedFcn',@(c,e) refreshPlot());

    % Save buttons
    btnSavePNG = uibutton(ctrlPanel,'push','Text','Save Figure PNG','Position',[10 5 140 24],...
        'ButtonPushedFcn',@(btn,event) savePNG());
    btnSaveMAT = uibutton(ctrlPanel,'push','Text','Save Data MAT','Position',[150 5 140 24],...
        'ButtonPushedFcn',@(btn,event) saveMAT());

    % Right panel: plotting
    axPanel = uipanel(fig,'Title','Map / Plot','Position',[320 10 860 680]);
    app.axPanel = axPanel;

    % Try to use geoplot (Mapping Toolbox). If not available, use regular axes.
    hasMapping = license('test','map_toolbox') && exist('geoplot','file')==2;
    if hasMapping
        ax = geoaxes(axPanel,'Position',[0.02 0.02 0.96 0.96]);
        title(ax,'Bistatic ellipse & antenna sector (geographic)');
    else
        axHandle = axes('Parent',axPanel,'Position',[0.06 0.06 0.92 0.92]); %#ok<LAXES>
        axis equal;
        grid on;
        title(axHandle,'Bistatic ellipse & antenna sector (ENU meters)');
        ax = axHandle;
    end
    app.ax = ax;
    app.hasMapping = hasMapping;

    % Attach app data to figure for callbacks
    guidata(fig,app);

    % Initial plot
    refreshPlot();

    % -------------------------
    % Nested callback functions
    % -------------------------
    function updatePos()
        app = guidata(fig);
        app.tx = [txLatEdit.Value, txLonEdit.Value];
        app.rx = [rxLatEdit.Value, rxLonEdit.Value];
        guidata(fig,app);
        refreshPlot();
    end

    function addEllipse()
        app = guidata(fig);
        r = rangeEdit.Value;
        % color selection cycle
        col = app.colors(app.nextColor,:);
        app.nextColor = app.nextColor + 1;
        if app.nextColor > size(app.colors,1)
            app.nextColor = 1;
        end
        % store ellipse
        eid = sprintf('R=%.1f m',r);
        new = struct('range',r,'color',col,'visible',true);
        app.ellipses{end+1} = new;
        % update list UI
        items = cellfun(@(s) sprintf('R=%.1f m',s.range), app.ellipses,'UniformOutput',false);
        ellipseList.Items = items;
        ellipseList.Value = items{end};
        guidata(fig,app);
        refreshPlot();
    end

    function toggleEllipse()
        app = guidata(fig);
        sel = ellipseList.Value;
        if isempty(sel), return; end
        % find index
        items = cellfun(@(s) sprintf('R=%.1f m',s.range), app.ellipses,'UniformOutput',false);
        idx = find(strcmp(items,sel),1);
        if isempty(idx), return; end
        % toggle visible
        app.ellipses{idx}.visible = ~app.ellipses{idx}.visible;
        % reflect in item text by appending [off]
        if app.ellipses{idx}.visible
            ellipseList.Items{idx} = sprintf('R=%.1f m',app.ellipses{idx}.range);
        else
            ellipseList.Items{idx} = sprintf('R=%.1f m [off]',app.ellipses{idx}.range);
        end
        guidata(fig,app);
        refreshPlot();
    end

    function removeEllipse()
        app = guidata(fig);
        sel = ellipseList.Value;
        if isempty(sel), return; end
        items = cellfun(@(s) sprintf('R=%.1f m',s.range), app.ellipses,'UniformOutput',false);
        idx = find(strcmp(items,sel),1);
        if isempty(idx), return; end
        app.ellipses(idx) = [];
        % update list
        if isempty(app.ellipses)
            ellipseList.Items = {};
        else
            ellipseList.Items = cellfun(@(s) sprintf('R=%.1f m',s.range), app.ellipses,'UniformOutput',false);
            ellipseList.Value = ellipseList.Items{1};
        end
        guidata(fig,app);
        refreshPlot();
    end

    function clearEllipses()
        app = guidata(fig);
        app.ellipses = {};
        ellipseList.Items = {};
        guidata(fig,app);
        refreshPlot();
    end

    function azChange(val)
        app = guidata(fig);
        app.az_center = val;
        azValLabel.Text = sprintf('Az = %.1f°',app.az_center);
        guidata(fig,app);
        refreshPlot();
    end

    function bwChange(val)
        app = guidata(fig);
        app.beamwidth = val;
        bwValLabel.Text = sprintf('BW = %.1f°',app.beamwidth);
        guidata(fig,app);
        refreshPlot();
    end

    function refreshPlot()
        % Recompute and redraw everything
        app = guidata(fig);
        ax = app.ax;

        % Clear axes
        if app.hasMapping
            cla(ax);
        else
            cla reset;
        end

        % Basic plotting: show TX and RX points
        tx_ll = app.tx;
        rx_ll = app.rx;

        if app.hasMapping
            geoplot(ax, rx_ll(1), rx_ll(2), 'kp','MarkerSize',12,'MarkerFaceColor','k'); hold(ax,'on');
            geoplot(ax, tx_ll(1), tx_ll(2), 'b^','MarkerSize',8,'MarkerFaceColor','b');
        else
            % convert positions to ENU, with RX as origin
            [txENU] = llh2enu_vector(tx_ll, rx_ll);
            plot(ax, 0, 0, 'kp','MarkerSize',12,'MarkerFaceColor','k'); hold(ax,'on'); % rx at origin
            plot(ax, txENU(1), txENU(2), 'b^','MarkerSize',8,'MarkerFaceColor','b'); % tx
        end

        % Draw each ellipse
        for k = 1:numel(app.ellipses)
            ell = app.ellipses{k};
            if ~ell.visible, continue; end
            Rb = ell.range;

            % compute ellipse in ENU (meters) with origin at RX
            txENU = llh2enu_vector(app.tx, app.rx); % [E N] relative to RX
            rxENU = [0,0];
            d_foci = norm(txENU - rxENU);
            if Rb < d_foci
                % no real ellipse - skip with warning marker
                % plot a red cross at midpoint to indicate invalid
                mid_ll = [(app.tx(1)+app.rx(1))/2, (app.tx(2)+app.rx(2))/2];
                if app.hasMapping
                    geoplot(ax, mid_ll(1), mid_ll(2), 'rx','MarkerSize',10);
                else
                    midENU = llh2enu_vector(mid_ll, app.rx);
                    plot(ax, midENU(1), midENU(2), 'rx','MarkerSize',10);
                end
                continue;
            end

            % ellipse params
            a = Rb/2; % semi-major
            c = d_foci/2;
            b = sqrt(max(a^2 - c^2,0));

            % ellipse in focal coordinates: center at midpoint
            centerENU = (txENU + rxENU)/2; % [E N]
            % angle of major axis (from ENU x-axis east to line pointing from RX->TX)
            dx = txENU(1) - rxENU(1);
            dy = txENU(2) - rxENU(2);
            phi = atan2(dy, dx); % radians (ENU: x east, y north)

            t = linspace(0,2*pi,app.nEllipsePoints);
            x_ = a*cos(t);
            y_ = b*sin(t);

            % rotate by phi and translate by center
            Rmat = [cos(phi) -sin(phi); sin(phi) cos(phi)];
            pts = (Rmat * [x_; y_])';
            pts(:,1) = pts(:,1) + centerENU(1);
            pts(:,2) = pts(:,2) + centerENU(2);

            % optionally mask by sector
            showSector = chkShowSector.Value;
            maskedIdx = true(size(pts,1),1);
            if showSector
                % compute bearing from RX (origin) to each point (deg clockwise from North)
                bearings = atan2_deg(pts(:,1), pts(:,2)); % function returns deg clockwise from N
                halfBW = app.beamwidth/2;
                azc = app.az_center;
                maskedIdx = angularWithin(bearings, azc-halfBW, azc+halfBW);
            end

            % Convert ellipse pts back to lat/lon for plotting
            latpts = zeros(size(pts,1),1);
            lonpts = zeros(size(pts,1),1);
            for i=1:size(pts,1)
                [latpts(i), lonpts(i)] = enu2ll(pts(i,1), pts(i,2), app.rx);
            end

            % Plot full ellipse (subtle color)
            if app.hasMapping
                geoplot(ax, latpts, lonpts, 'Color',ell.color, 'LineWidth',1.0);
            else
                plot(ax, pts(:,1), pts(:,2), 'Color', ell.color, 'LineWidth',1.0);
            end

            % Plot masked arc highlighted
            if app.hasMapping
                geoplot(ax, latpts(maskedIdx), lonpts(maskedIdx), 'Color',ell.color,'LineWidth',3);
            else
                plot(ax, pts(maskedIdx,1), pts(maskedIdx,2), 'Color', ell.color,'LineWidth',3);
            end
        end

        % Plot antenna sector wedge (from RX center)
        if chkShowSector.Value
            plotSector(ax, app.az_center, app.beamwidth, 2000, app);
        end

        hold(ax,'off');
        % update stored app
        guidata(fig,app);
    end

    function savePNG()
        app = guidata(fig);
        [file,path] = uiputfile('bistatic_plot.png','Save figure as PNG');
        if isequal(file,0), return; end
        fname = fullfile(path,file);
        % capture figure
        exportgraphics(app.fig,fname,'Resolution',300);
        uialert(app.fig,'Figure saved as PNG.','Saved');
    end

    function saveMAT()
        app = guidata(fig);
        [file,path] = uiputfile('bistatic_data.mat','Save data as MAT');
        if isequal(file,0), return; end
        fname = fullfile(path,file);

        % Prepare export: tx, rx, ellipses, last plotted points
        data.tx = app.tx;
        data.rx = app.rx;
        data.azimuth = app.az_center;
        data.beamwidth = app.beamwidth;
        data.ellipses = app.ellipses;
        save(fname,'data');
        uialert(app.fig,'Data saved as MAT.','Saved');
    end

    % -------------------------
    % Helper / geometry functions
    % -------------------------
    function b = atan2_deg(E,N)
        % return bearing in degrees clockwise from North given ENU east (E) and north (N)
        % atan2 returns angle from x-axis (east), we convert
        % compass = mod(90 - rad2deg(atan2(N,E)),360); alternative formula.
        % But simpler: bearing = atan2(E,N) in radians, convert to degrees and mod 360.
        b = rad2deg(atan2(E,N));
        b = mod(b + 360, 360);
    end

    function inside = angularWithin(angles, a1, a2)
        % Check if angles (deg) are within sector a1..a2 (deg), handles wrap-around.
        angles = mod(angles,360);
        a1 = mod(a1,360);
        a2 = mod(a2,360);
        if a1 <= a2
            inside = (angles >= a1) & (angles <= a2);
        else
            inside = (angles >= a1) | (angles <= a2);
        end
    end

    function [lat, lon] = enu2ll(E,N, origin_ll)
        % convert ENU coordinates (E,N) in meters to lat/lon (deg)
        % using local tangential approximation via ECEF conversion
        % origin_ll = [lat_deg, lon_deg] (deg)
        % altitude assumed 0
        % Strategy: convert origin to ECEF; compute ECEF = origin + [-]; convert back.
        lat0 = origin_ll(1); lon0 = origin_ll(2); h0 = 0;
        % Convert origin to ECEF
        [x0,y0,z0] = llh2ecef(lat0, lon0, h0);
        % ENU to ECEF
        phi = deg2rad(lat0); lambda = deg2rad(lon0);
        t = [ -sin(lambda)           cos(lambda)          0;
              -sin(phi)*cos(lambda) -sin(phi)*sin(lambda) cos(phi);
               cos(phi)*cos(lambda)  cos(phi)*sin(lambda) sin(phi) ];
        ecef_vec = t' * [E; N; 0]; % 3x1 in ECEF frame
        x = x0 + ecef_vec(1);
        y = y0 + ecef_vec(2);
        z = z0 + ecef_vec(3);
        [lat,lon,~] = ecef2llh(x,y,z);
    end

    function enu = llh2enu_vector(ll, origin_ll)
        % Compute ENU [E N] of a lat/lon point ll relative to origin_ll
        % returns [E N]
        [x,y,z] = llh2ecef(ll(1), ll(2), 0);
        [x0,y0,z0] = llh2ecef(origin_ll(1), origin_ll(2), 0);
        dx = x - x0; dy = y - y0; dz = z - z0;
        phi = deg2rad(origin_ll(1)); lambda = deg2rad(origin_ll(2));
        R = [ -sin(lambda)           cos(lambda)          0;
             -sin(phi)*cos(lambda) -sin(phi)*sin(lambda) cos(phi);
              cos(phi)*cos(lambda)  cos(phi)*sin(lambda) sin(phi) ];
        enu3 = R * [dx; dy; dz];
        enu = [enu3(1), enu3(2)];
    end

    function [x,y,z] = llh2ecef(lat_deg, lon_deg, h)
        % Convert geodetic lat/lon/h to ECEF (WGS84)
        % Inputs in degrees, meters.
        a = 6378137.0; % WGS84 semi-major
        f = 1/298.257223563;
        e2 = 2*f - f^2;
        lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
        N = a ./ sqrt(1 - e2 .* (sin(lat)).^2);
        x = (N + h) .* cos(lat) .* cos(lon);
        y = (N + h) .* cos(lat) .* sin(lon);
        z = (N .* (1 - e2) + h) .* sin(lat);
    end

    function [lat,lon,h] = ecef2llh(x,y,z)
        % Convert ECEF to lat/lon/h using iterative method (WGS84)
        a = 6378137.0;
        f = 1/298.257223563;
        e2 = 2*f - f^2;
        b = a * (1 - f);
        ep2 = (a^2 - b^2) / b^2;

        p = sqrt(x.^2 + y.^2);
        theta = atan2(z * a, p * b);
        lon = atan2(y, x);
        lat = atan2(z + ep2 * b * sin(theta).^3, p - e2 * a * cos(theta).^3);
        N = a ./ sqrt(1 - e2 .* sin(lat).^2);
        h = p ./ cos(lat) - N;

        lat = rad2deg(lat);
        lon = rad2deg(lon);
    end

    function plotSector(ax, az_center, beamwidth, radius, app)
        % plots a sector (wedge) centered at RX origin onto axes ax
        % radius in meters for ENU plotting; if mapping, we sample points and convert to latlon
        half = beamwidth/2;
        t = linspace(-half, +half, 200) + az_center; % degrees clockwise from North
        % convert bearing to ENU vector: E = r*sin(bearing), N = r*cos(bearing)
        br = deg2rad(t);
        E = radius * sin(br);
        N = radius * cos(br);
        if app.hasMapping
            latpts = zeros(length(E),1); lonpts = zeros(length(E),1);
            for i=1:length(E)
                [latpts(i), lonpts(i)] = enu2ll(E(i), N(i), app.rx);
            end
            geoplot(ax, latpts, lonpts, 'k--', 'LineWidth',1.5);
            % fill sector lightly
            geoplot(ax, [app.rx(1); latpts; app.rx(1)], [app.rx(2); lonpts; app.rx(2)], 'k-');
        else
            plot(ax, [0 E 0], [0 N 0], 'k--','LineWidth',1.2);
            % fill
            patch(ax, [0;E';0], [0;N';0], [0.9 0.9 0.9], 'FaceAlpha',0.2,'EdgeColor','none');
        end
    end

end

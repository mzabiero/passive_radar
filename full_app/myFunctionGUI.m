function passive_radar_app
    % === MAIN CONFIGURATION & LAYOUT CONSTANTS ===
    config.figPos   = [100 100 1600 900]; % Mniejsza domyślna, żeby testować skalowanie
    config.gap      = 10;
    
    % Panel Dimensions (Fixed Widths)
    layout.leftW    = 380;
    layout.rightW   = 250;
    % Center width calculated dynamically later
    
    % === HEIGHT CALCULATIONS (Dynamic) ===
    % Ustawiamy wysokości lewych paneli jako procent dostępnej wysokości
    totalH = config.figPos(4);
    leftAvailableH = totalH - 4*config.gap; 
    
    layout.hFiles   = floor(leftAvailableH * 0.40); % 40% na parametry
    layout.hActions = floor(leftAvailableH * 0.35); % 35% na przyciski
    layout.hStatus  = leftAvailableH - layout.hFiles - layout.hActions; % Reszta na historię
    
    % Heights - Right Panel
    layout.hSimParam      = 360;
    layout.hSimAlgorithms = 280;
    layout.hSimStatus     = 220;

    % Heights - Center Panel
    layout.hClim    = 110;
    
    % === GLOBAL DATA STRUCTURE ===
    data = struct();
    data.history = {};
    data.histTitles = {};
    data.simulation = struct();
    data.simulation.hist = {};
    data.simulation.active = struct('x_ref',0, 'x_surv',0, 'x_echo', 0);
    data.simulation.sim_lines = {};
    data.log_lines = {};
    
    % Parallel Pool Check
    if isempty(gcp('nocreate'))
        parpool('threads');
    end

    % === UI CREATION ===
    fig = uifigure('Name','Passive Radar App','Position',config.figPos);
    
    % Recalculate center width based on actual figure size
    layout.centerW  = fig.Position(3) - layout.leftW - layout.rightW - 4*config.gap;

    % --- Left Column: Parameters & Actions ---
    [pFiles, pActions, pStatus] = createLeftPanel(fig, layout, config);
    
    % --- Right Column: Simulation ---
    [pSimParam, pSimAlg, pSimStatus] = createRightPanel(fig, layout, config);
    
    % --- Center Column: Plotting ---
    [ax, terminal, sliderMin, sliderMax, chkAuto, chkFixed, editCmin, editCmax] = createCenterPanel(fig, layout, config);

    % --- Init Log ---
    logTerminal('App Initialized', 'N');

    % =====================================================================
    %                       HELPER FUNCTIONS FOR UI
    % =====================================================================
    
    function [pFiles, pActions, pStatus] = createLeftPanel(parent, l, c)
        % Calculate Y positions (bottom-up)
        yStatus  = c.gap;
        yActions = yStatus + l.hStatus + c.gap;
        yFiles   = yActions + l.hActions + c.gap;
        
        % Panels
        pFiles   = uipanel(parent, 'Title','Parameters',       'Position',[c.gap yFiles l.leftW l.hFiles], 'Scrollable','on');
        pActions = uipanel(parent, 'Title','Algorithms',       'Position',[c.gap yActions l.leftW l.hActions], 'Scrollable','on');
        pStatus  = uipanel(parent, 'Title','Status / Workspace','Position',[c.gap yStatus l.leftW l.hStatus], 'Scrollable','on');
        
        % -- File Pickers (Top of Parameters) --
        % Umieszczamy je na samej górze "wirtualnej" przestrzeni
        % (Zostaną obsłużone w setupProcessingParams jako startowy offset)
        
        % -- Parameters Generation --
        % Funkcja teraz sama zarządza pozycją guzików "Choose file" wewnątrz scrolla
        setupProcessingParams(pFiles);
        
        % -- Action Buttons --
        btns = {
            'CAF - raw',         @(s,e) runCAF('orig');
            'CAF - filtr close', @(s,e) runCAF('close');
            'CAF - filtr wide',  @(s,e) runCAF('wide');
            'CLEAN + CAF',       @(s,e) runCLEAN();
            'CAF (Normal)',      @(s,e) runCAF('Normal');
            'Plot Spectrum',     @(s,e) plotSpectrum();
            'Correlation',       @(s,e) runXcorr();
            'Plot Time',         @(s,e) runPlotTime()
        };
        
        % Dynamic layout for Actions
        btnH = 30; gap = 10;
        contentH = numel(btns) * (btnH + gap) + 50; % Total height needed
        startY = max(l.hActions - 40, contentH); % Start from top
        
        currentY = startY;
        for i = 1:size(btns, 1)
            uibutton(pActions, 'Text', btns{i,1}, 'Position', [20 currentY 250 btnH], 'ButtonPushedFcn', btns{i,2});
            currentY = currentY - (btnH + gap);
            if i == 3 
                 uicheckbox(pActions,'Text','backward','Position',[275 currentY+(btnH+gap) 100 btnH], ...
                     'ValueChangedFcn', @(src,~) setParam('back_filt', src.Value, 'data.params'));
            end
        end
        
        % -- Status Components --
        createStatusPanelContent(pStatus, l.leftW, l.hStatus);
    end

    function setupProcessingParams(parentPanel)
        % File Buttons at the top
        topMargin = 40;
        
        % Param Definitions
        % Format: Label, Default, FieldName, Type, Options/Extras (must exist for all rows)
        paramDefs = {
            'fs (Hz)', 10e6, 'fs', 'num', [];
            'fc (Hz)', 650e6, 'fc', 'num', [];
            'Max delay', 200, 'max_delay', 'num', [];
            'Backward delay', 0, 'backward_delay', 'num', [];
            'Doppler bins', 512, 'doppler_bins', 'num', [];
            'R (Decimation)', 2750, 'R', 'num', [];
            'FiltNear: Order', 4, 'filtOrder_close', 'num', [];
            'Forget Fact (Near)', 0.99, 'forgetting_fact_close', 'num', [];
            'BlockLen (Near)', 8192, 'blockLength_close', 'num', [];
            'FiltWide: Order', 500, 'filtOrder_wide', 'num', [];
            'Forget Fact (Wide)', 0.99999, 'forgetting_fact_wide', 'num', [];
            'BlockLen (Wide)', 8192, 'blockLength_wide', 'num', [];
            'Backward Filtering', 0, 'back_filt', 'num', [];
            'Window Type', 'none', 'window_type', 'drop', {'none','hamming', 'hann', 'blackmann', 'kaiser'}
        };
        
        rowH = 26;
        numParams = size(paramDefs, 1);
        
        % Obliczamy całkowitą wysokość potrzebną na kontrolki + guziki plików
        totalContentHeight = topMargin + (numParams * (rowH + 5)) + 20;
        
        % Jeśli panel jest mniejszy niż treść, startujemy od wyliczonej wysokości
        % żeby scrollbar działał od góry.
        currentY = max(parentPanel.Position(4), totalContentHeight) - 40;
        
        % File buttons
        uibutton(parentPanel, 'Text','Choose ref',  'Position',[10 currentY 160 30], 'ButtonPushedFcn', @(~,~) loadFile('ref'));
        uibutton(parentPanel, 'Text','Choose surv', 'Position',[190 currentY 160 30], 'ButtonPushedFcn', @(~,~) loadFile('surv'));
        
        currentY = currentY - 40; % Gap after buttons
        
        data.params = struct(); 
        
        for i = 1:numParams
            lbl = paramDefs{i,1};
            def = paramDefs{i,2};
            fld = paramDefs{i,3};
            typ = paramDefs{i,4};
            opts = paramDefs{i,5};
            
            data.params.(fld) = def; 
            
            uilabel(parentPanel, 'Text', lbl, 'Position', [10 currentY 150 22], 'HorizontalAlignment', 'left');
            
            if strcmp(typ, 'num')
                uieditfield(parentPanel, 'numeric', 'Value', def, ...
                    'Position', [190 currentY 160 22], ...
                    'ValueChangedFcn', @(src,~) setParam(fld, src.Value, 'data.params'));
            elseif strcmp(typ, 'drop')
                uidropdown(parentPanel, 'Items', opts, ...
                    'Position', [190 currentY 160 22], ...
                    'ValueChangedFcn', @(src,~) setParam(fld, src.Value, 'data.params'));
            end
            currentY = currentY - 30; % Move down
        end
    end

    function createStatusPanelContent(parent, w, h)
        % Layout constants inside Status Panel
        margin = 10;
        btnHeight = 30;
        textAreaHeight = 100; % Fixed height for text info
        
        % Calculate Listbox Height dynamically to fill remaining space
        % h is total panel height.
        % We need: Top Margin + TextArea + Gap + Listbox + Gap + Buttons + Bottom Margin
        % ListboxH = h - (10 + 100 + 10 + 30 + 10 + 20 title space)
        
        listboxY = margin + btnHeight + margin; 
        listboxH = h - textAreaHeight - btnHeight - 4*margin - 20;
        if listboxH < 50, listboxH = 50; end % Minimum safeguard
        
        % Text Area (Top)
        uitextarea(parent, 'Position', [margin listboxY+listboxH+margin w-2*margin textAreaHeight], ...
            'Editable', 'off', 'Tag', 'txtStatus');
        
        % History List (Middle - filling space)
        uilabel(parent, 'Text', 'History:', 'Position', [margin listboxY+listboxH w-2*margin 20]);
        uilistbox(parent, 'Position', [margin listboxY w-2*margin listboxH], 'Tag', 'lstHistory');
        
        % Buttons (Bottom)
        btnY = margin;
        btnW = (w - 4*margin) / 3;
        
        uibutton(parent, 'Text','Load',  'Position',[margin btnY btnW btnHeight],  'ButtonPushedFcn', @(~,~) loadHistory());
        uibutton(parent, 'Text','Delete','Position',[margin+btnW+margin btnY btnW btnHeight], 'ButtonPushedFcn', @(~,~) delHistory());
        uibutton(parent, 'Text','Reset', 'Position',[margin+2*(btnW+margin) btnY btnW btnHeight], 'ButtonPushedFcn', @(~,~) rstHistory());
    end

    function [pSimParam, pSimAlg, pSimStatus] = createRightPanel(parent, l, c)
        rightX = parent.Position(3) - l.rightW - c.gap;
        
        ySimStatus  = c.gap + 100;
        ySimButtons = ySimStatus + l.hSimStatus + c.gap;
        ySimParam   = ySimButtons + l.hSimAlgorithms + c.gap;
        
        pSimParam   = uipanel(parent, 'Title','Sim Parameters',   'Position',[rightX ySimParam l.rightW l.hSimParam]);
        pSimAlg     = uipanel(parent, 'Title','Sim Algorithms',   'Position',[rightX ySimButtons l.rightW l.hSimAlgorithms]);
        pSimStatus  = uipanel(parent, 'Title','Sim Workspace',    'Position',[rightX ySimStatus l.rightW l.hSimStatus]);
        
        setupSimParams(pSimParam, l.rightW, l.hSimParam, c.gap);
        
        uibutton(pSimAlg, 'Text','Add echo',            'Position',[c.gap l.hSimAlgorithms-60 l.rightW-2*c.gap 30], 'ButtonPushedFcn', @(~,~) addEcho());
        uibutton(pSimAlg, 'Text','Save signals to files','Position',[c.gap l.hSimAlgorithms-100 l.rightW-2*c.gap 30], 'ButtonPushedFcn', @(~,~) saveSimToFiles());
        uibutton(pSimAlg, 'Text','Plot Pure Echo (Sim)', 'Position',[c.gap l.hSimAlgorithms-140 l.rightW-2*c.gap 30], 'ButtonPushedFcn', @(~,~) plotPureEcho(), 'FontWeight','bold');

        uibutton(parent, 'Text','Refresh',           'Position',[rightX+c.gap 45+c.gap l.rightW-2*c.gap 30], 'ButtonPushedFcn', @(~,~) refreshSimWorkspace(pSimStatus));
        uibutton(parent, 'Text','Set signals active','Position',[rightX+c.gap 15 l.rightW-2*c.gap 30],       'ButtonPushedFcn', @(~,~) setHistActive());
    end

    function setupSimParams(parent, w, h, gap)
        simDefaults = struct(...
            'fs', 10e6, 'fc', 650e6, 'Cyclic_Prefix', '1/4', 'OFDM_Mode', '2k', ...
            'Duration', 0.1, 'Echo_position', 2000, 'Echo_velocity', 100, ...
            'Attenuation', 0.5, 'DPI', 0, 'Clutter', 0, 'File_Name', 'sig_sim_iq');
        
        data.simulation.params = simDefaults;
        data.simulation.params.add_echo_counter = 1;
        
        fields = fieldnames(simDefaults);
        y = h - 50;
        lblW = w/2 - gap;
        
        for i = 1:numel(fields)
            fld = fields{i};
            val = simDefaults.(fld);
            
            if strcmp(fld, 'Cyclic_Prefix')
                uilabel(parent,'Text',fld,'Position',[gap y lblW 22]);
                uidropdown(parent, 'Items', {'0','1/4','1/8','1/16','1/32'}, 'Value', val, ...
                    'Position',[lblW+gap y lblW 22], 'ValueChangedFcn', @(s,~) setParam(fld, s.Value, 'data.simulation.params'));
            elseif strcmp(fld, 'OFDM_Mode')
                uilabel(parent,'Text',fld,'Position',[gap y lblW 22]);
                uidropdown(parent, 'Items', {'2k','8k'}, 'Value', val, ...
                    'Position',[lblW+gap y lblW 22], 'ValueChangedFcn', @(s,~) setParam(fld, s.Value, 'data.simulation.params'));
            elseif strcmp(fld, 'DPI') || strcmp(fld, 'Clutter')
                uicheckbox(parent, 'Text', fld, 'Position', [gap y lblW 22], ...
                    'ValueChangedFcn', @(s,~) setParam(fld, s.Value, 'data.simulation.params'));
            elseif strcmp(fld, 'File_Name')
                uilabel(parent,'Text',fld,'Position',[gap y lblW 22]);
                uieditfield(parent, 'text', 'Value', val, 'Position',[lblW+gap y lblW 22], ...
                    'ValueChangedFcn', @(s,~) setParam(fld, s.Value, 'data.simulation.params'));
            else
                uilabel(parent,'Text',fld,'Position',[gap y lblW 22]);
                uieditfield(parent, 'numeric', 'Value', val, 'Position',[lblW+gap y lblW 22], ...
                    'ValueChangedFcn', @(s,~) setParam(fld, s.Value, 'data.simulation.params'));
            end
            y = y - 26;
        end
    end

    function [ax, terminal, sMin, sMax, cAuto, cFixed, eMin, eMax] = createCenterPanel(parent, l, c)
        centerX   = l.leftW + 2*c.gap;
        
        yClim    = c.gap;
        yAxes    = yClim + l.hClim + c.gap;
        hAxes    = parent.Position(4) - yAxes - c.gap;
        terminalW = l.centerW * 0.2;
        
        pCenter = uipanel(parent, "Title", "Plotting Panel", ...
            "Position", [centerX c.gap l.centerW l.hClim+hAxes+c.gap]);
        
        % --- FIX: AXES MARGINS ---
        % Zamiast wpychać osie na styk, dajemy im oddech
        axMarginLeft = 50;  % Miejsce na label Osi Y
        axMarginBottom = 50; % Miejsce na label Osi X
        axW = l.centerW - c.gap - axMarginLeft - 20; % 20 zapasu z prawej
        axH = hAxes - c.gap - axMarginBottom - 20;   % 20 zapasu z góry
        
        ax = uiaxes(pCenter, 'Position', [axMarginLeft axMarginBottom+l.hClim axW axH]);
        title(ax,'CAF'); xlabel(ax,'Bistatic velocity (m/s)'); ylabel(ax,'Bistatic range (km)');
        
        % Clim Control Panel
        pClim = uipanel(pCenter, 'Title','Scaling C axis', ...
            'Position', [c.gap c.gap l.centerW-(4*c.gap)-terminalW l.hClim]);
            
        uilabel(pClim,'Text','C-min','Position',[10 60 50 22]);
        eMin = uieditfield(pClim,'numeric','Value',-30, 'Position',[60 60 80 22]);
        uilabel(pClim,'Text','C-max','Position',[160 60 50 22]);
        eMax = uieditfield(pClim,'numeric','Value',0, 'Position',[210 60 80 22]);
        
        uibutton(pClim,'Text','Update','Position',[310 60 80 22], 'ButtonPushedFcn', @(~,~) setCLimManual());
        
        uilabel(pClim,'Text','Min slider','Position',[420 60 70 22]);
        sMin = uislider(pClim,'Position',[500 70 220 3], 'Limits',[-80 0],'Value',-30, 'ValueChangedFcn', @(~,~) updateCLimFromSliders());
        
        uilabel(pClim,'Text','Max slider','Position',[420 20 70 22]);
        sMax = uislider(pClim,'Position',[500 30 220 3], 'Limits',[-80 0],'Value',0, 'ValueChangedFcn', @(~,~) updateCLimFromSliders());
        
        cAuto = uicheckbox(pClim,'Text','Auto (mean→max)', 'Position',[740 60 150 22],'Value',true, 'ValueChangedFcn', @(~,~) updateCAFScaling());
        cFixed = uicheckbox(pClim,'Text','C axis fixed (mean→0)', 'Position',[740 30 150 22],'Value',false, 'ValueChangedFcn', @(~,~) updateCAFScaling());
        
        % Terminal Panel
        pTerm = uipanel(pCenter, 'Title','Log', 'Position',[l.centerW-terminalW-c.gap c.gap terminalW l.hClim]);
        terminal = uitextarea(pTerm, 'Position', [0 0 terminalW l.hClim-20], 'Editable','off');
    end

    % =====================================================================
    %                       LOGIC & CALLBACKS
    % =====================================================================

    % --- File Handling ---
    function loadFile(type)
        [file, path] = uigetfile({'*.*'});
        if isequal(file,0), return; end
        filename = fullfile(path, file);
        
        try
            if endsWith(file, '.mat', 'IgnoreCase', true)
                loaded = load(filename);
                sig = struct2array(loaded);
            else
                sig = read_complex_binary(filename);
            end
            sig = sig / rms(sig);
            
            if strcmp(type,'ref')
                data.ref = sig;
                data.ref_filename = file;
            else
                data.surv = sig;
                data.lastSurv = sig;
                data.surv_filename = file;
            end
            updateStatusText();
            logTerminal(sprintf('Loaded %s: %s', type, file), 'A');
        catch ME
            uialert(fig, ME.message, 'Load Error');
        end
    end

    % --- Parameter Setter ---
    function setParam(name, val, destStr)
        if strcmp(destStr, 'data.params')
            data.params.(name) = val;
        elseif strcmp(destStr, 'data.simulation.params')
            data.simulation.params.(name) = val;
        end
        updateStatusText();
    end

    % --- CAF Execution ---
    function runCAF(mode)
        logTerminal(sprintf('CAF Running (%s)...', mode),'N');
        
        if ~isfield(data,'ref') || ~isfield(data,'surv')
            uialert(fig,'Load signals first!','Error'); return;
        end
        p = data.params;
        
        if strcmp(mode,'orig') || ~isfield(data,'lastSurv')
            surv_in = data.surv;
        else
            surv_in = data.lastSurv;
        end
        
        switch mode
            case 'orig'
                surv_clean = data.surv;
            case 'close'
                surv_clean = clutter_removal(data.ref, surv_in, p.filtOrder_close, p.forgetting_fact_close, p.blockLength_close, p.back_filt);
            case 'wide'
                surv_clean = clutter_removal(data.ref, surv_in, p.filtOrder_wide, p.forgetting_fact_wide, p.blockLength_wide, p.back_filt);
            otherwise
                surv_clean = surv_in;
        end
        
        [caf, d_ax, v_ax] = CAF(data.ref, surv_clean, p.fs, p.fc, p.max_delay, p.doppler_bins, p.R, p.window_type, p.backward_delay);
        
        data.lastCAF = caf;
        data.lastSurv = surv_clean;
        data.delay_axis = d_ax;
        data.doppler_axis = v_ax;
        
        plotCAF(caf, d_ax, v_ax, ['CAF: ' mode]);
        
        maxP = max(caf(:));
        meanP = mean(caf(:));
        sigP  = pow2db(mean(abs(surv_clean).^2));
        
        logTerminal(sprintf('Max power: %.2f dB', maxP), 'N');
        logTerminal(sprintf('Mean power: %.2f dB', meanP), 'A');
        logTerminal(sprintf('Signal power: %.2f dB', sigP), 'A');
        
        axStruct.delay = d_ax; axStruct.doppler = v_ax;
        addToHistory(caf, surv_clean, p.window_type, mode, axStruct);
        updateStatusText();
    end

    function runCLEAN()
        if ~isfield(data,'lastCAF')
            uialert(fig,'Calculate CAF first!','Error'); return;
        end
        p = data.params;
        [x_clean, r_km, v_ms] = CLEAN(data.lastCAF, data.ref, data.lastSurv, p.fs, p.fc, p.max_delay, p.doppler_bins, p.R);
        
        msg = sprintf('Object found:\nRange = %.2f km\nVelocity = %.2f m/s\n\nRemove?', r_km, v_ms);
        choice = questdlg(msg,'CLEAN','Yes','No','No');
        
        if strcmp(choice,'Yes')
            data.lastSurv = x_clean;
            [caf, d_ax, v_ax] = CAF(data.ref, data.lastSurv, p.fs, p.fc, p.max_delay, p.doppler_bins, p.R, p.window_type, p.backward_delay);
            
            data.lastCAF = caf;
            plotCAF(caf, d_ax, v_ax, 'CAF after CLEAN');
            
            axStruct.delay = d_ax; axStruct.doppler = v_ax;
            addToHistory(caf, x_clean, p.window_type, 'CLEAN', axStruct);
            logTerminal(sprintf('Target removed: R=%.2fkm V=%.2fm/s', r_km, v_ms), 'A');
        end
    end

    % --- Plotting Helpers ---
    function plotCAF(caf, delay_axis, doppler_axis, ttl)
        if isempty(delay_axis)
            delay_axis = 1:size(caf,1);
            doppler_axis = 1:size(caf,2);
        end
        imagesc(ax, doppler_axis, delay_axis, caf); 
        axis(ax,'xy');
        colormap(ax,'jet'); colorbar(ax);
        title(ax, ttl);
        updateCAFScaling(); 
    end

    function updateCAFScaling()
        if ~isfield(data,'lastCAF'), return; end
        caf = data.lastCAF;
        if chkAuto.Value
            ax.CLim = [mean(caf(:)), max(caf(:))];
        elseif chkFixed.Value
            ax.CLim = [mean(caf(:)), 0];
        else
            ax.CLim = [editCmin.Value, editCmax.Value];
        end
    end

    function setCLimManual()
        ax.CLim = [editCmin.Value, editCmax.Value];
        chkAuto.Value = false; chkFixed.Value = false;
    end
    
    function updateCLimFromSliders()
        ax.CLim = [sliderMin.Value, sliderMax.Value];
        chkAuto.Value = false; chkFixed.Value = false;
    end

    function plotSpectrum()
        figure('Name','Signal Spectrum');
        tiledlayout(2,1);
        nexttile;
        if isfield(data,'ref'), plot_spectrum(data.ref, data.params.fs); title("Ref Spectrum"); end
        nexttile;
        if isfield(data,'lastSurv'), plot_spectrum(data.lastSurv, data.params.fs); title("Surv Spectrum"); end
    end

    function runXcorr()
        if ~isfield(data,'lastSurv') || ~isfield(data,'ref'), return; end
        [corr, lags] = xcorr(data.lastSurv, data.ref, data.params.max_delay);
        corr = abs(corr)/max(abs(corr));
        D = 3e8 * lags ./ data.params.fs;
        figure; plot(D/1000, mag2db(abs(corr))); title("Cross-correlation"); xlabel('Dist (km)'); ylabel('dB');
    end

    function runPlotTime()
        if ~isfield(data,'ref') || ~isfield(data,'surv'), return; end
        N = length(data.ref);
        t = linspace(0, N/data.params.fs, N);
        figure; 
        subplot(2,1,1); plot(t, data.ref); title("Ref Time");
        subplot(2,1,2); plot(t, data.lastSurv); title("Surv Time");
    end

    % --- HISTORY MANAGEMENT ---
    function addToHistory(caf, surv_state, w_type, tag, axStruct)
        entry = struct();
        entry.caf = caf;
        entry.surv = surv_state;
        entry.window_type = w_type;
        entry.mode = tag;
        entry.axes = axStruct;
        entry.ref_filename = safeStr(data, 'ref_filename');
        entry.surv_filename = safeStr(data, 'surv_filename');
        
        data.history{end+1} = entry;
        data.histTitles{end+1} = sprintf('%d: %s', numel(data.history), tag);
        
        lst = findobj(fig, 'Tag', 'lstHistory');
        lst.Items = data.histTitles;
    end

    function loadHistory()
        lst = findobj(fig, 'Tag', 'lstHistory');
        idx = lst.Value;
        if isempty(idx), return; end
        
        pos = find(strcmp(lst.Items, idx));
        if isempty(pos), return; end
        
        if (isempty(data.history))
            return;
        else
            entry = data.history{pos};
        end
        data.lastCAF = entry.caf;
        data.lastSurv = entry.surv;
        data.lastWindow_type = entry.window_type;
        
        plotCAF(entry.caf, entry.axes.delay, entry.axes.doppler, ['History: ' entry.mode]);
        updateStatusText();
        
        logTerminal('--- History Loaded ---', 'N');
        logTerminal(['Ref: ' entry.ref_filename], 'A');
        logTerminal(['Surv: ' entry.surv_filename], 'A');
        
        maxP = max(entry.caf(:));
        meanP = mean(entry.caf(:));
        sigP = pow2db(mean(abs(entry.surv).^2));
        
        logTerminal(sprintf('Max power: %.2f dB', maxP), 'A');
        logTerminal(sprintf('Mean power: %.2f dB', meanP), 'A');
        logTerminal(sprintf('Signal power: %.2f dB', sigP), 'A');
    end

    function delHistory()
        lst = findobj(fig, 'Tag', 'lstHistory');
        idx = lst.Value;
        if isempty(idx), return; end
        pos = find(strcmp(lst.Items, idx));
        if isempty(pos), return; end
        
        data.history(pos) = [];
        data.histTitles(pos) = [];
        lst.Items = data.histTitles;
    end

    function rstHistory()
        data.history = {};
        data.histTitles = {};
        % findobj(data, 'Tag', 'lstHistory').Items = {};
        if isfield(data,'surv'), data.lastSurv = data.surv; end
        updateStatusText();

    end

    % --- STATUS UPDATES ---
    function updateStatusText()
        lines = {};
        if isfield(data,'ref')
            lines{end+1} = sprintf('Ref: %s (%d samps)', safeStr(data,'ref_filename'), length(data.ref));
        else
            lines{end+1} = 'Ref: N/A';
        end
        if isfield(data,'surv')
            lines{end+1} = sprintf('Surv: %s (%d samps)', safeStr(data,'surv_filename'), length(data.surv));
        else
            lines{end+1} = 'Surv: N/A';
        end
        
        if isfield(data, 'lastCAF')
            lambda = freq2wavelen(data.params.fc);
            T = length(data.ref)/data.params.fs;
            lines{end+1} = sprintf('Res: %.1f m | %.2f m/s', 3e8/data.params.fs, lambda/T);
            lines{end+1} = sprintf('Dur: %.3f s', T);
        end
        
        txt = findobj(fig, 'Tag', 'txtStatus');
        txt.Value = lines;
    end

    function logTerminal(info, type)
        if strcmp(type, 'N')
            data.log_lines = {info};
        else
            data.log_lines{end+1} = info;
        end
        terminal.Value = data.log_lines;
    end

    % === SIMULATION LOGIC ===
    
    function sim_sig = simulationGenSig()
        p = data.simulation.params;
        switch p.OFDM_Mode
            case '2k', Nfft = 2048;
            case '8k', Nfft = 8192;
        end
        
        cp_frac = str2num(p.Cyclic_Prefix); 
        if isempty(cp_frac), cp_frac = 0; end
        cpLen = Nfft * cp_frac;
        
        sim_sig = runDVBTsim(p.fs, cpLen, p.Duration, p.OFDM_Mode);
    end

    function addEcho()
        
        p = data.simulation.params;
        counter = p.add_echo_counter;
        
        if data.simulation.active.x_ref == 0
             data.simulation.hist{counter}.x_ref = simulationGenSig();
             raw_signal = data.simulation.hist{counter}.x_ref;
             base_name = p.File_Name;
        else
             raw_signal = data.simulation.active.x_ref;
             base_name = [data.simulation.active.name '_2'];
        end
        
        if p.Clutter
             N_clut = 1000;
             c_range = sort(1 + rand(1, N_clut)*30e3);
             c_mag = (1./c_range); c_mag = c_mag/max(c_mag);
             clutter = create_clutter(c_range, c_mag);
        else
             clutter = 0;
        end
        
        range_m = p.Echo_position;
        vel_ms  = p.Echo_velocity;
        atten   = p.Attenuation;
        
        range_m = [300 300 300];
        vel_ms = [100 400 -200];
        atten = [0.1 0.0001 0.0001];
        
        [x_ref, x_surv, x_echo_pure] = simulate_target_ref_surv_signals(...
            raw_signal, p.fs, range_m, vel_ms, p.fc, atten, p.DPI, clutter);
            
        if data.simulation.active.x_ref ~= 0
             x_surv = x_surv .* data.simulation.active.x_surv;
        end
        
        data.simulation.hist{counter}.name = base_name;
        data.simulation.hist{counter}.x_ref = x_ref;
        data.simulation.hist{counter}.x_surv = x_surv;
        data.simulation.hist{counter}.x_echo = x_echo_pure;
        
        data.simulation.x_echo = x_echo_pure; 
        
        updateSimWorkspaceLines();
        data.simulation.params.add_echo_counter = counter + 1;
        logTerminal(sprintf('Echo added: R=%.0fm V=%.0fm/s', range_m, vel_ms), 'A');
    end

    function plotPureEcho()
        % === FIX: Bezpieczniejsze sprawdzanie dostępności danych ===
        
        % Sprawdź czy "aktywny" sygnał echa jest wektorem (ma dane), czy tylko domyślnym zerem
        hasActiveData = numel(data.simulation.active.x_echo) > 1;
        
        % Sprawdź czy ostatni element historii posiada dane echa
        hasHistData = ~isempty(data.simulation.hist) && ...
                      isfield(data.simulation.hist{end}, 'x_echo') && ...
                      numel(data.simulation.hist{end}.x_echo) > 1;

        % Jeśli nie ma danych ani tu, ani tu -> błąd
        if ~hasActiveData && ~hasHistData
             uialert(fig, 'No echo generated yet.', 'Info'); return;
        end
        
        % === Wybór sygnału do wyświetlenia ===
        if hasActiveData
             sig = data.simulation.active.x_echo;
             nm = 'Active Echo';
        else
             sig = data.simulation.hist{end}.x_echo;
             nm = ['Echo from ' data.simulation.hist{end}.name];
        end
        
        fft_bins = data.params.doppler_bins;
        % Rysowanie
        figure('Name', 'Pure Echo Signal');
        subplot(3,1,1); plot(real(sig)); title([nm ' (Real)']); grid on;
        subplot(3,1,2); plot(abs(sig)); title('Magnitude'); grid on;
        subplot(3,1,3); plot(linspace(-fft_bins/2,fft_bins/2,fft_bins),fftshift(fft(abs(sig),fft_bins)));
    end

    function saveSimToFiles()
        if data.simulation.active.x_ref == 0
             uialert(fig,'No active simulation to save','Error'); return;
        end
        saveDir = 'data/simulation';
        if ~exist(saveDir,'dir')
            mkdir(saveDir);
        end
        
        sim_save_del(data.simulation.active.name, ...
            data.simulation.active.x_ref, data.simulation.active.x_surv, saveDir);
        logTerminal(['Saved sim signals to ' saveDir], 'A');
    end

    function updateSimWorkspaceLines()
        delete(pSimStatus.Children); 
        
        yStart = layout.hSimStatus - 40;
        hItem = 30;
        
        for k = 1:numel(data.simulation.hist)
            item = data.simulation.hist{k};
            yPos = yStart - (k-1)*(hItem+5);
            
            uicheckbox(pSimStatus, 'Text', item.name, 'Position', [10 yPos 150 22], ...
                'Value', false, 'Tag', num2str(k), ...
                'ValueChangedFcn', @(src,~) setHistActiveUI(src, k));
            
            uibutton(pSimStatus, 'Text', 'X', 'Position', [170 yPos 30 22], ...
                'ButtonPushedFcn', @(~,~) deleteSimItem(k));
        end
    end

    function setHistActiveUI(src, idx)
        chks = findobj(pSimStatus, 'Type', 'uicheckbox');
        for i=1:numel(chks)
            if chks(i) ~= src, chks(i).Value = false; end
        end
        
        if src.Value
            data.simulation.active = data.simulation.hist{idx};
            logTerminal(['Active Sim: ' data.simulation.hist{idx}.name], 'A');
        else
            data.simulation.active = struct('x_ref',0,'x_surv',0, 'x_echo',0);
        end
    end
    
    function setHistActive()
       logTerminal('Select active signal via checkboxes in Sim Workspace.', 'A');
    end

    function deleteSimItem(idx)
        data.simulation.hist(idx) = [];
        updateSimWorkspaceLines();
    end

    function refreshSimWorkspace(parent)
        delete(parent.Children);
        data.simulation.hist = {};
        data.simulation.active = struct('x_ref',0,'x_surv',0,'x_echo',0);
        data.simulation.params.add_echo_counter = 1;
        logTerminal('Sim workspace cleared', 'A');
    end

    function s = safeStr(d, f)
        if isfield(d,f), s = d.(f); else, s = 'n/a'; end
    end
end

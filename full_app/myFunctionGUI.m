function passive_radar_app

    % === Layout constants ===
    figPos   = [100 100 2000 900];
    gap      = 10;
    vert_gap = 20;
    
    % left panel
    leftW    = 380;
        hFiles   = 360;              
        hActions = 240;
        hStatus  = 260;
        
        yStatus  = gap;
        yActions = yStatus + hStatus + gap;
        yFiles   = yActions + hActions + gap;
    
    % Right panel
    rightW = 250;
    rightX = figPos(3) - rightW - gap;
        hSimParam   = 360;
        hSimAlgorithms = 240;
        hSimStatus  = 260;

        ySimStatus  = gap;
        ySimButtons = ySimStatus + hSimStatus + gap;
        ySimParam   = ySimButtons + hSimAlgorithms + gap;

    % Middle panel
    centerX   = leftW + 2*gap;
    centerW   = figPos(3) - leftW - rightW- 4*gap;
        hClim    = 110;
        yClim    = gap;
        yAxes    = yClim + hClim + gap;
        hAxes    = figPos(4) - yAxes - gap;
        
   
    % === Main window ===
    fig = uifigure('Name','Passive Radar App','Position',figPos);
    
    % Global data
    data = struct();
    data.history = {};
    data.histTitles = {};
    data.simulation = {};
    
    
    %--------------------------------------------------------------%
    % ================== Left column panels ============================
    
    pFiles = uipanel(fig,'Title','Parameters','Position',[gap yFiles leftW hFiles]);
    pActions = uipanel(fig,'Title','Algorithms','Position',[gap yActions leftW hActions]);
    pStatus = uipanel(fig,'Title','Status / Workspace','Position',[gap yStatus leftW hStatus]);
    
                    % === File pickers ===
    uibutton(pFiles,'Text','Choose ref',...
        'Position',[10 hFiles-60 160 30],...
        'ButtonPushedFcn',@(src,event) loadFile('ref'));
    uibutton(pFiles,'Text','Choose surv',...
        'Position',[190 hFiles-60 160 30],...
        'ButtonPushedFcn',@(src,event) loadFile('surv'));

                    % === Parameters ===
    labels = {'fs (Hz)','fc (Hz)','max_delay','doppler_bins','R',...
              'FiltClose: Order','Step','BlockLen',...
              'FiltWide: Order','Step','BlockLen'};
    defaults = {10e6, 650e6, 200, 512, 100,...
                10, 7e-2, 8192,...
                170, 5e-4, 8192};
    data.params = cell2struct(defaults, ...
        {'fs','fc','max_delay','doppler_bins','R',...
         'filtOrder_close','stepSize_close','blockLength_close',...
         'filtOrder_wide','stepSize_wide','blockLength_wide'},2);

    y = hFiles-100;
    fields = fieldnames(data.params);
    for i=1:numel(fields)
        uilabel(pFiles,'Text',labels{i},'Position',[10 y 150 22], 'HorizontalAlignment','left');
        uieditfield(pFiles,'numeric','Value',defaults{i},...
            'Position',[190 y 160 22],...
            'ValueChangedFcn',@(src,event) setParam(fields{i},src.Value,'data.params'));
        y = y-26;
        if y < 10, break; end
    end

                    % === Action buttons ===
    uibutton(pActions,'Text','CAF - raw','Position',[20 hActions-60 340 30],...
        'ButtonPushedFcn',@(src,event) runCAF('orig'));
    uibutton(pActions,'Text','CAF - filtr close','Position',[20 hActions-100 340 30],...
        'ButtonPushedFcn',@(src,event) runCAF('close'));
    uibutton(pActions,'Text','CAF - filtr wide','Position',[20 hActions-140 340 30],...
        'ButtonPushedFcn',@(src,event) runCAF('wide'));
    uibutton(pActions,'Text','CLEAN + CAF','Position',[20 hActions-180 340 30],...
        'ButtonPushedFcn',@(src,event) runCLEAN());
    uibutton(pActions,'Text','Plot Spectrum','Position',[20 hActions-220 340 30],...
        'ButtonPushedFcn',@(src,event) plotSpectrum());
    
                    % Status textarea
    txtStatus = uitextarea(pStatus,'Position',[10 10 leftW-20 hStatus-40],...
        'Editable','off');

                    % Historia wyników (listbox + przyciski)
    lstHistory = uilistbox(pStatus,'Position',[10 hStatus-210 leftW-20 85]);
    btnLoadHist = uibutton(pStatus,'Text','Load',...
        'Position',[15 15 80 30],...
        'ButtonPushedFcn',@(src,event) loadHistory());
    btnDelHist = uibutton(pStatus,'Text','Delete',...
        'Position',[100 15 80 30],...
        'ButtonPushedFcn',@(src,event) delHistory());

    % =================== Center column =====================
    centerP = uipanel(fig,"Title","Plotting Panel","Position",[leftW+2*gap gap centerW hClim+hAxes+gap]);
    ax = uiaxes(centerP,'Position',[gap gap+hClim centerW-(4*gap) hAxes-gap]);
    title(ax,'CAF');
    xlabel(ax,'Bistatic velocity (m/s)');
    ylabel(ax,'Bistatic range (km)');

    pClim = uipanel(centerP,'Title','Scaling C axis',...
        'Position',[gap gap centerW-(4*gap) hClim]);

                    % === CLim controls ===
    uilabel(pClim,'Text','C-min','Position',[10 60 50 22]);
    editCmin = uieditfield(pClim,'numeric','Value',-30,...
        'Position',[60 60 80 22]);
    uilabel(pClim,'Text','C-max','Position',[160 60 50 22]);
    editCmax = uieditfield(pClim,'numeric','Value',0,...
        'Position',[210 60 80 22]);
    btnUpdateCLim = uibutton(pClim,'Text','Update','Position',[310 60 80 22],...
        'ButtonPushedFcn',@(src,event) setCLimManual());

    uilabel(pClim,'Text','Min slider','Position',[420 60 70 22]);
    sliderMin = uislider(pClim,'Position',[500 70 220 3],...
        'Limits',[-80 0],'Value',-30);
    uilabel(pClim,'Text','Max slider','Position',[420 20 70 22]);
    sliderMax = uislider(pClim,'Position',[500 30 220 3],...
        'Limits',[-80 0],'Value',0);
    chkAuto = uicheckbox(pClim,'Text','Auto (mean→0)',...
        'Position',[740 60 150 22],'Value',true);


   % ================= Right column: Simulation panels ======================
    pSimParam   = uipanel(fig,'Title','Simulation Parameters','Position',[rightX ySimParam rightW hSimParam]);
    pSimAlg = uipanel(fig,'Title','Simulation Algorithms','Position',[rightX ySimButtons rightW hSimAlgorithms]);
    pSimStatus  = uipanel(fig,'Title','Simulation Workspace','Position',[rightX ySimStatus rightW hSimStatus]);
    
                    % --- Simulation parameters ---
    sim_labels = {'fs (Hz)','fc (Hz)','Cyclic Prefix','OFDM Mode','Duration (s)',...
              'Echo position (m)','Echo velocity (m/s)','Attenuation','DPI','Clutter'};
    sim_defaults = {10e6, 650e6, '1/4', '2k', 0.1, ...
                2000, 100, 0.5, 0, 0};
    data.simulation.params = struct( ...
        'fs', 10e6, ...
        'fc', 650e6, ...
        'Cyclic_Prefix','1/4', ...   % store as ratio string
        'OFDM_Mode','2k', ...        % store as string
        'Duration',0.1, ...
        'Echo_position',2000, ...
        'Echo_velocity',100, ...
        'Attenuation',0.5, ...
        'DPI', 0, ...
        'Clutter',0);  % default save path
    data.simulation.x_ref;
    data.simulation.x_surv;
    sim_fields = fieldnames(data.simulation.params);

    y =hSimParam-50;
    labelW = rightW/2 - gap;
    editFieldW = labelW;
    
    for i=1:numel(sim_fields)
        switch sim_fields{i}
            case 'Cyclic_Prefix'
                uilabel(pSimParam,'Text',sim_labels{i},'Position',[gap y labelW 22], ...
                    'HorizontalAlignment','left');
                uidropdown(pSimParam, ...
                    'Items',{'0','1/4','1/8','1/16','1/32'}, ...
                    'Value',sim_defaults{3}, ...
                    'Position',[labelW+gap y editFieldW 22], ...
                    'ValueChangedFcn',@(src,event) setParam('Cyclic_Prefix',src.Value,'data.simulation.params'));   
            case 'OFDM_Mode'
                uilabel(pSimParam,'Text',sim_labels{i},'Position',[gap y labelW 22], ...
                    'HorizontalAlignment','left');
                uidropdown(pSimParam, ...
                    'Items',{'2k','8k'}, ...
                    'Value',sim_defaults{4}, ...
                    'Position',[labelW+gap y editFieldW 22], ...
                    'ValueChangedFcn',@(src,event) setParam('OFDM_Mode',src.Value,'data.simulation.params'));
            case 'DPI'
                uicheckbox(pSimParam,'Text',sim_labels{i},'Position',[gap y labelW 22],...
                    'ValueChangedFcn',@(src,event) setParam(sim_fields{i},src.Value,'data.simulation.params'));
            case 'Clutter'
                uicheckbox(pSimParam,'Text',sim_labels{i},'Position',[gap y labelW 22], ...
                    'ValueChangedFcn',@(src,event) setParam(sim_fields{i},src.Value,'data.simulation.params'));
            otherwise
                uilabel(pSimParam,'Text',sim_labels{i},'Position',[gap y labelW 22], ...
                    'HorizontalAlignment','left');
                uieditfield(pSimParam,'numeric','Value',sim_defaults{i}, ...
                    'Position',[labelW+gap y editFieldW 22], ...
                    'ValueChangedFcn',@(src,event) setParam(sim_fields{i},src.Value,'data.simulation.params'));
        end
        y = y-26;
        if y < 10, break; end
    end
    
    data.simulation.params.SaveFile = 'data//mat/DVBT_signals/sig_sim_iq.bin';

    uibutton(pSimParam,'Text','Select Output File', ...
    'Position',[gap 10 rightW-2*gap 30], ...
    'ButtonPushedFcn',@(src,event) saveFile());

    uibutton(pSimAlg,'Text','Generate simulated signal', ...
    'Position',[gap hSimAlgorithms-60 rightW-2*gap 30], ...
    'ButtonPushedFcn',@(src,event) simulationGenSig());

    uibutton(pSimAlg,'Text','Add echo', ...
    'Position',[gap hSimAlgorithms-90-gap rightW-2*gap 30], ...
    'ButtonPushedFcn',@(src,event) addEcho());
    uibutton(pSimAlg,'Text','Save signals to files', ...
    'Position',[gap hSimAlgorithms-120-2*gap rightW-2*gap 30], ...
    'ButtonPushedFcn',@(src,event) saveSimToFiles());

    simWorkspace = uitextarea(pSimStatus,'Position',[0 0 rightW hSimStatus-2*gap]);

%-----------------------------------------------------------------------------------------%
    % ===== Nested functions =====

    % === File Handling ===
    function loadFile(type)
        [file,path] = uigetfile({'*.*'});
        if isequal(file,0), return; end
        filename = fullfile(path,file);
        sig = read_complex_binary(filename);
        sig = sig / rms(sig);
        if strcmp(type,'ref')
            data.ref = sig;
            data.ref_filename = file;
        else
            data.surv = sig;
            data.surv_filename = file;
        end
        updateStatus();
        %plot_spectrum(data.ref,data.params.fs);
    end
    function saveFile()
        [file,path] = uiputfile('*.bin','Save simulated signal as',...
            data.simulation.params.SaveFile);
        if isequal(file,0)
            disp('User canceled file selection');
        else
            data.simulation.params.SaveFile = fullfile(path,file);
            fprintf("Save file set to: %s\n", data.simulation.params.SaveFile);
        end
    end

    function saveSimToFiles()
        sim_save_del(data.simulation.params.SaveFile, ...
            data.simulation.x_ref, data.simulation.x_surv);
    end
    % === Parameters handling
    function setParam(name,val,dest)
        switch dest
            case 'data.params'
                data.params.(name) = val;
            case 'data.simulation.params'
                data.simulation.params.(name) = val;
        end
        updateStatus();
    end
        
    % === CAF Algorithms
    function runCAF(mode)
        if ~isfield(data,'ref') || ~isfield(data,'surv')
            uialert(fig,'Załaduj pliki!','Błąd'); return;
        end
        fs = data.params.fs;
        fc = data.params.fc;
        max_delay = data.params.max_delay;
        doppler_bins = data.params.doppler_bins;
        R = data.params.R;

        if strcmp(mode,'orig') || ~isfield(data,'lastSurv')
            surv_in = data.surv;
        else
            surv_in = data.lastSurv;
        end

        switch mode
            case 'orig'
                surv_clean = data.surv;
            case 'close'
                surv_clean = clutter_removal(data.ref,surv_in,...
                    data.params.filtOrder_close,...
                    data.params.stepSize_close,...
                    data.params.blockLength_close);
            case 'wide'
                surv_clean = clutter_removal(data.ref,surv_in,...
                    data.params.filtOrder_wide,...
                    data.params.stepSize_wide,...
                    data.params.blockLength_wide);
        end

        [caf, delay_axis, doppler_axis] = CAF(data.ref,surv_clean,fs,fc,max_delay,doppler_bins,R);
        
        plotCAF(caf,delay_axis,doppler_axis,['CAF: ' mode]);
    
        data.lastCAF = caf;
        data.lastSurv = surv_clean;
        data.delay_axis = delay_axis;
        data.doppler_axis = doppler_axis;
        addToHistory(caf,surv_clean,mode);

        updateStatus();
    end

    function runCLEAN()
        if ~isfield(data,'lastCAF')
            uialert(fig,'Najpierw uruchom CAF!','Błąd'); return;
        end
        [x_clean, r_km, v_ms] = CLEAN(data.lastCAF, data.ref, data.lastSurv,...
            data.params.fs, data.params.fc, data.params.max_delay,...
            data.params.doppler_bins, data.params.R);

        msg = sprintf('Object found:\n\nRange = %.2f km\nVelocity = %.2f m/s\n\nRemove?',...
                      r_km, v_ms);
        choice = questdlg(msg,'CLEAN','Yes','No','No');

        if strcmp(choice,'Yes')
            data.lastSurv = x_clean;
            [caf, delay_axis, doppler_axis] = CAF(data.ref,x_clean,...
                data.params.fs, data.params.fc,...
                data.params.max_delay, data.params.doppler_bins, data.params.R);
            plotCAF(caf,delay_axis,doppler_axis,'CAF after CLEAN iteration');
            data.lastCAF = caf;
            data.delay_axis = delay_axis;
            data.doppler_axis = doppler_axis;
            addToHistory(caf,x_clean,'CLEAN');
        end
        updateStatus();
    end
    
    function plotSpectrum()
        if ~isfield(data,'ref') && ~isfield(data,'surv')
            uialert(fig,'Choose signal','Błąd'); return;
        end
       
        choice = questdlg('Signal Spectrum','Choose Signal','ref','surv','ref');
    
        if (strcmp(choice, 'ref'))
            if isfield(data,'ref')
                plot_spectrum(data.ref,data.params.fs);
            else
                uialert(fig,'Choose signal','Błąd'); return;
            end
        elseif(strcmp(choice,'surv')) 
            if isfield(data,'lastSurv')
                plot_spectrum(data.lastSurv,data.params.fs);
            elseif isfield(data, 'surv')
                plot_spectrum(data.surv,data.params.fs);
            else
                uialert(fig,'Choose signal','Błąd'); return;
            end
        else
            return;
        end
    end
    
    % === History management ===
    function addToHistory(caf,surv_state,tag)
        entry.caf = caf;
        entry.surv = surv_state;
        entry.mode = tag;
        
        data.history{end+1} = entry;
        data.histTitles{end+1} = sprintf('%d: %s',numel(data.history),tag);
        lstHistory.Items = data.histTitles;
    end

    function loadHistory()
        idx = lstHistory.Value;
        if isempty(idx), return; end
        % znajdź indeks
        pos = find(strcmp(lstHistory.Items,idx));
        if isempty(pos), return; end
        entry = data.history{pos};
        caf = entry.caf;
        surv_state = entry.surv;
        mode = entry.mode;
        
        % ustaw jako bieżące
        data.lastCAF = caf;
        data.lastSurv = surv_state;
        plotCAF(caf,data.delay_axis,data.doppler_axis,'History: ' + string(mode));
        updateStatus();
    end

    function delHistory()
        idx = lstHistory.Value;
        if isempty(idx), return; end
        pos = find(strcmp(lstHistory.Items,idx));
        if isempty(pos), return; end
        data.history(pos) = [];
        data.histTitles(pos) = [];
        lstHistory.Items = data.histTitles;
    end
    
    % === Simulation helper ===
    function simulationGenSig()
        fprintf("Simulation function called\n");
    
        % Map cyclic prefix string → numeric length
        switch data.simulation.params.OFDM_Mode
            case '2k'
                Nfft = 2048;
            case '8k'
                Nfft = 8192;
        end
    
        switch data.simulation.params.Cyclic_Prefix
            case '0',    cpLen = 0;
            case '1/4',  cpLen = Nfft/4;
            case '1/8',  cpLen = Nfft/8;
            case '1/16', cpLen = Nfft/16;
            case '1/32', cpLen = Nfft/32;
        end
    
        sim_sig = runDVBTsim(data.simulation.params.fs, ...
            cpLen, ...
            data.simulation.params.Duration, ...
            data.simulation.params.OFDM_Mode);
        data.simulation.data = sim_sig;
        updateSimWorkspace();
    end
    
    function addEcho()

        params = data.simulation.params;
        fname = params.SaveFile;
        fs = data.simulation.params.fs;
        range_m = data.simulation.params.Echo_position;
        velocity_ms = data.simulation.params.Echo_velocity;
        fc = data.simulation.params.fc;
        atten = data.simulation.params.Attenuation;
        raw_signal = data.simulation.data;
        dpi = data.simulation.params.DPI;
        clutter = data.simulation.params.Clutter;

        [x_ref, x_surv] = simulate_target_ref_surv_signals(raw_signal,fs, ...
            range_m,velocity_ms,fc,atten,fname,dpi,clutter);
        data.simulation.x_ref = x_ref;
        data.simulation.x_surv = x_surv;
        
        fprintf("Add echo called\n");
        updateSimWorkspace();
    end

    % === Plot helper ===
    function plotCAF(caf,delay_axis,doppler_axis,ttl)
        if nargin < 2 || isempty(delay_axis)
            delay_axis = 1:size(caf,1);
            doppler_axis = 1:size(caf,2);
        end
        imagesc(ax,doppler_axis,delay_axis,caf); axis(ax,'xy');
        colormap(ax,'jet'); colorbar(ax);
        if chkAuto.Value
            ax.CLim = [mean(caf(:)), 0];
        else
            ax.CLim = [editCmin.Value, editCmax.Value];
        end
        title(ax,ttl);
        disp("Done");
    end

    function updateStatus()
        lines = {};
        if isfield(data,'ref')
            lines{end+1} = sprintf('Ref file: %s  (%d samples)', ...
                safeStr(data,'ref_filename'), length(data.ref));
        else
            lines{end+1} = 'Ref file: not loaded';
        end
        if isfield(data,'surv')
            lines{end+1} = sprintf('Surv file: %s (%d samples)', ...
                safeStr(data,'surv_filename'), length(data.surv));
        else
            lines{end+1} = 'Surv file: not loaded';
        end
        if isfield(data,'lastCAF')
            sz = size(data.lastCAF);
            lines{end+1} = sprintf('CAF size: %d x %d',sz(1),sz(2));
        else
            lines{end+1} = 'CAF: not computed yet';
        end
        txtStatus.Value = lines;
    end

    function updateSimWorkspace()
        lines = {};
        if isfield(data.simulation, 'x_ref')
            lines{1} = sprintf('Simulated signal: %f  (%d samples)', ...
                    data.simulation.params.fs, length(data.simulation.x_ref));
        else
            lines{1} = 'Surv file: not loaded';
        end
            simWorkspace.Value = lines;
    end
    function s = safeStr(d,field)
        if isfield(d,field), s = d.(field); else, s = '(n/a)'; end
    end

    % === CLim helpers ===
    function setCLimManual()
        ax.CLim = [editCmin.Value editCmax.Value];
        chkAuto.Value = false;
    end
        
    sliderMin.ValueChangedFcn = @(~,~) updateCLimFromSliders();
    sliderMax.ValueChangedFcn = @(~,~) updateCLimFromSliders();
    chkAuto.ValueChangedFcn   = @(~,~) updateCAFScaling();
    
    function updateCAFScaling()
        if chkAuto.Value && isfield(data,'lastCAF')
            ax.CLim = [mean(data.lastCAF(:)) 0];
        end
    end
    
    function updateCLimFromSliders()
        ax.CLim = [sliderMin.Value sliderMax.Value];
        chkAuto.Value = false;
    end

end
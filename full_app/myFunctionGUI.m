function passive_radar_app
    % === Layout constants ===
    figPos   = [100 100 2000 1000];
    gap      = 10;
    vert_gap = 20;
    
    % left panel
    leftW    = 380;
        hFiles   = 360;
        hActions = 320;
        hActions_scroll = 60;
        hStatus  = 280;
        
        yStatus  = gap;
        yActions = yStatus + hStatus + gap;
        yFiles   = yActions + hActions + gap;
    
    % Right panel
    rightW = 250;
    rightX = figPos(3) - rightW - gap;
        hSimParam   = 360;
        hSimAlgorithms = 240;
        hSimStatus  = 260;

        ySimStatus  = gap+100;
        ySimButtons = ySimStatus + hSimStatus + gap;
        ySimParam   = ySimButtons + hSimAlgorithms + gap;

    % Middle panel
    centerX   = leftW + 2*gap;
    centerW   = figPos(3) - leftW - rightW- 4*gap;
        hClim    = 110;
        yClim    = gap;
        terminalW= centerW*(1/5);
        terminalH= hClim;
        yAxes    = yClim + hClim + gap;
        hAxes    = figPos(4) - yAxes - gap;
        
   
    % === Main window ===
    fig = uifigure('Name','Passive Radar App','Position',figPos);

    % Global data
    data = struct();
    data.history = {};
    data.histTitles = {};
    data.simulation = {};
    
    % Initialization of parallel computing and simulation
    if isempty(gcp('nocreate'))   % check if pool exists
        parpool('threads');       % or 'local' cluster, or specify #workers
    end

    %--------------------------------------------------------------%
    % ================== Left column panels ============================
    
    pFiles = uipanel(fig,'Title','Parameters','Position',[gap yFiles leftW hFiles], 'Scrollable','on');
    pActions = uipanel(fig,'Title','Algorithms','Position',[gap yActions leftW hActions], 'Scrollable','on');
    pStatus = uipanel(fig,'Title','Status / Workspace','Position',[gap yStatus leftW hStatus],'Scrollable','on');

    pActions.Scrollable = 'on';
    
                    % === File pickers ===
    uibutton(pFiles,'Text','Choose ref',...
        'Position',[10 hFiles-10 160 30],...
        'ButtonPushedFcn',@(src,event) loadFile('ref'));
    uibutton(pFiles,'Text','Choose surv',...
        'Position',[190 hFiles-10 160 30],...
        'ButtonPushedFcn',@(src,event) loadFile('surv'));

                    % === Parameters ===
    labels = {'fs (Hz)','fc (Hz)','max_delay','doppler_bins','R',...
              'FiltNear: Order','Forgetting Factor','BlockLen',...
              'FiltWide: Order','Forgetting Factor','BlockLen','Window Type'};
    defaults = {10e6, 650e6, 200, 512, 2750,...
                4, 0.99, 8192,...
                500, 0.99999, 8192,0};
    data.params = cell2struct(defaults, ...
        {'fs','fc','max_delay','doppler_bins','R',...
         'filtOrder_close','forgetting_fact_close','blockLength_close',...
         'filtOrder_wide','forgetting_fact_wide','blockLength_wide','window_type'},2);
    win_list = {'none','hamming', 'hann', 'blackmann', 'kaiser'};

    y = hFiles-40;
    fields = fieldnames(data.params);
    for i=1:numel(fields)
        if strcmpi(labels{i},'Window Type')
            uilabel(pFiles,'Text',labels{i},'Position',[10 y 150 22], 'HorizontalAlignment','left');
            uidropdown(pFiles,'Items',win_list,...
            'Position',[190 y 160 22],...
            'ValueChangedFcn',@(src,event) setParam(fields{i},src.Value,'data.params'));
            y = y-26;
        else
        uilabel(pFiles,'Text',labels{i},'Position',[10 y 150 22], 'HorizontalAlignment','left');
        uieditfield(pFiles,'numeric','Value',defaults{i},...
            'Position',[190 y 160 22],...
            'ValueChangedFcn',@(src,event) setParam(fields{i},src.Value,'data.params'));
        y = y-26;
        end
    end

   
    % ==================== Action buttons =================================
    uibutton(pActions,'Text','CAF - raw','Position',[20 hActions+10 250 30],...
        'ButtonPushedFcn',@(src,event) runCAF('orig'));
    uibutton(pActions,'Text','CAF - filtr close','Position',[20 hActions-30 250 30],...
        'ButtonPushedFcn',@(src,event) runCAF('close'));
    uibutton(pActions,'Text','CAF - filtr wide','Position',[20 hActions-70 250 30],...
        'ButtonPushedFcn',@(src,event) runCAF('wide'));
    chk_wide = uicheckbox(pActions,'Text','backward','Position',[275 hActions-70 100 30]);
    uibutton(pActions,'Text','CLEAN + CAF','Position',[20 hActions-110 250 30],...
        'ButtonPushedFcn',@(src,event) runCLEAN());
    uibutton(pActions,'Text','CAF','Position',[20 hActions-150 250 30],...
        'ButtonPushedFcn',@(src,event) runCAF('Normal'));
    uibutton(pActions,'Text','Plot Spectrum','Position',[20 hActions-190 250 30],...
        'ButtonPushedFcn',@(src,event) plotSpectrum());
    uibutton(pActions,'Text','Correlation','Position',[20 hActions-230 250 30],...
        'ButtonPushedFcn',@(src,event) runXcorr());
    uibutton(pActions,'Text','Plot time','Position',[20 hActions-270 250 30],...
        'ButtonPushedFcn',@(src,event) runPlotTime()); 
     
                    % Status textarea
    txtStatus = uitextarea(pStatus,'Position',[10 10 leftW-20 hStatus-40],...
        'Editable','off');

                    % Results history
    lstHistory = uilistbox(pStatus,'Position',[10 hStatus-210 leftW-20 85]);
    btnLoadHist = uibutton(pStatus,'Text','Load',...
        'Position',[15 15 80 30],...
        'ButtonPushedFcn',@(src,event) loadHistory());
    btnDelHist = uibutton(pStatus,'Text','Delete',...
        'Position',[100 15 80 30],...
        'ButtonPushedFcn',@(src,event) delHistory());
    btnDelHist = uibutton(pStatus,'Text','Reset',...
        'Position',[185 15 80 30],...
        'ButtonPushedFcn',@(src,event) rstHistory());

    % =================== Center column: Plotting scene ===================
               
    centerP = uipanel(fig,"Title","Plotting Panel",...
        "Position", [leftW+2*gap gap centerW hClim+hAxes+gap]);
    ax = uiaxes(centerP,'Position',[gap gap+hClim centerW-yAxes hAxes-gap]);
    title(ax,'CAF');
    xlabel(ax,'Bistatic velocity (m/s)');
    ylabel(ax,'Bistatic range (km)');

    pClim = uipanel(centerP,'Title','Scaling C axis',...
        'Position',[gap gap centerW-(4*gap)-terminalW hClim]);

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
    chkAuto = uicheckbox(pClim,'Text','Auto (mean→max)',...
        'Position',[740 60 150 22],'Value',true);
    chkFixed = uicheckbox(pClim,'Text','C axis fixed (mean→0)',...
        'Position',[740 30 150 22],'Value',false);

                    % === Terminal panel ===
    pterminal = uipanel(centerP,'title','log window',...
        'position',[centerW-(4*gap)-terminalW+2*gap gap terminalW terminalH]);
    terminal = uitextarea(pterminal,'position',[0 0 terminalW terminalH-2*gap],...
       'editable','off');      
    
    % Initialization of log panel
    data.log_lines = {};
    data.log_lines = logTerminal(sprintf('Initialize'),'N',data.log_lines);

   % ================= Right column: Simulation panels ====================
    pSimParam   = uipanel(fig,'Title','Simulation Parameters',...
        'Position',[rightX ySimParam rightW hSimParam]);
    pSimAlg = uipanel(fig,'Title','Simulation Algorithms',...
        'Position',[rightX ySimButtons rightW hSimAlgorithms]);
    pSimStatus  = uipanel(fig,'Title','Simulation Workspace',...
        'Position',[rightX ySimStatus rightW hSimStatus]);
    

    
                    % --- Simulation parameters ---
    sim_labels = {'fs (Hz)','fc (Hz)','Cyclic Prefix',...
              'OFDM Mode','Duration (s)',...
              'Echo position (m)','Echo velocity (m/s)',...
              'Attenuation','DPI','Clutter', 'File name'};
    sim_defaults = {10e6, 650e6, '1/4', '2k', 0.1, ...
                2000, 100, 0.5, 0, 0, 'sig_sim_iq'};
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
        'Clutter',0, ...
        'File_Name', 'sig_sim_iq');  % default save path
    data.simulation.x_ref = 0;
    data.simulation.x_surv = 0;
    data.simulation.hist = {};
    data.simulation.active = struct('x_ref',0,'x_surv',0);
    sim_fields = fieldnames(data.simulation.params);

    y = hSimParam-50;
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
            case 'File_Name'
                uilabel(pSimParam,'Text',sim_labels{i},'Position',[gap y labelW 22], ...
                    'HorizontalAlignment','left');
                uieditfield(pSimParam,'Text','Value',sim_defaults{i}, ...
                    'Position',[labelW+gap y editFieldW 22], ...
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
   
        data.simulation.params.add_echo_counter = 1;

        uibutton(pSimAlg,'Text','Add echo', ...
        'Position',[gap hSimAlgorithms-60 rightW-2*gap 30], ...
        'ButtonPushedFcn',@(src,event) addEcho());
        uibutton(pSimAlg,'Text','Save signals to files', ...
        'Position',[gap hSimAlgorithms-90-gap rightW-2*gap 30], ...
        'ButtonPushedFcn',@(src,event) saveSimToFiles());
        
        uibutton(fig,'Text','Refresh','Position',[rightX+gap 45+gap rightW-2*gap 30],...
        'ButtonPushedFcn',@(src,event) refreshSimWorkspace(pSimStatus));
        uibutton(fig,'Text','Set signals active','Position',[rightX+gap 15 rightW-2*gap 30],...
        'ButtonPushedFcn',@(src,event) setHistActive());
   
        
%-------------------------------------------------------------------------%
                    % ===== Nested functions =====

                    % === File Handling ===
    function loadFile(type)
        [file,path] = uigetfile({'*.*'});
        if isequal(file,0), return; end
        filename = fullfile(path,file);
        if endsWith(file, '.mat', 'IgnoreCase', true)
            sig = struct2array(load(filename));
        else
            sig = read_complex_binary(filename);
        end

        sig = sig / rms(sig);
        if strcmp(type,'ref')
            data.ref = sig;
            data.ref_filename = file;
        else
            data.surv = sig;
            data.surv_filename = file;
        end
        updateStatus();

    end
    
    function saveFile()
        [file,path] = uiputfile('*.bin','Save simulated signal as',...
            data.simulation.params.File_Name);
        if isequal(file,0)
            disp('User canceled file selection');
        else
            data.simulation.params.File_Name = fullfile(path,file);
            fprintf("Save file set to: %s\n", data.simulation.params.File_Name);
        end
    end

    function saveSimToFiles()
        
        saveDir = 'data/simulation';
        sim_save_del(data.simulation.active.name, ...
            data.simulation.active.x_ref, data.simulation.active.x_surv,saveDir);
        fprintf("Signals saved\n");
    end

                    % === Parameters handling ===
    function setParam(name,val,dest)
        switch dest
            case 'data.params'
                data.params.(name) = val;
            case 'data.simulation.params'
                data.simulation.params.(name) = val;
        end
        updateStatus();
    end
        
                    % === CAF Algorithms ===
    function runCAF(mode)

        data.log_lines = logTerminal(sprintf('CAF Running ...'),'N', data.log_lines);
        
        if ~isfield(data,'ref') || ~isfield(data,'surv')
            uialert(fig,'Załaduj pliki!','Błąd'); return;
        end
        fs = data.params.fs;
        fc = data.params.fc;
        max_delay = data.params.max_delay;
        doppler_bins = data.params.doppler_bins;
        R = data.params.R;
        window_type = data.params.window_type;

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
                    data.params.forgetting_fact_close,...
                    data.params.blockLength_close);
            case 'wide'
                if chk_wide.Value
                    x_rev = clutter_removal(flipud(data.ref), flipud(surv_in),...
                    data.params.filtOrder_wide,...
                    data.params.forgetting_fact_wide,...
                    data.params.blockLength_wide);
                    surv_clean = flipud(x_rev);
                else
                    surv_clean = clutter_removal(data.ref,surv_in,...
                    data.params.filtOrder_wide,...
                    data.params.forgetting_fact_wide,...
                    data.params.blockLength_wide);
                end
                
            otherwise
                surv_clean = surv_in;
        end
        
        [caf, delay_axis, doppler_axis] = CAF(data.ref,surv_clean,fs,fc,max_delay,doppler_bins,R,window_type);
        
        plotCAF(caf,delay_axis,doppler_axis,['CAF: ' mode]);
    
        data.lastCAF = caf;
        data.lastSurv = surv_clean;
        data.delay_axis = delay_axis;
        data.doppler_axis = doppler_axis;
        axes.delay = delay_axis;
        axes.doppler = doppler_axis;
        addToHistory(caf,surv_clean,window_type,mode,axes);

        updateStatus();
        data.log_lines = logTerminal(sprintf('Max power: %.2f dB',max(data.lastCAF(:))),'N',data.log_lines);
        data.log_lines = logTerminal(sprintf('Mean power: %.2f dB',mean(data.lastCAF(:))),'A',data.log_lines);
    end
 
    function runCLEAN()
        if ~isfield(data,'lastCAF')
            uialert(fig,'First calculate CAF!','ERROR'); return;
        end
        [x_clean, r_km, v_ms] = CLEAN(data.lastCAF, data.ref, data.lastSurv,...
            data.params.fs, data.params.fc, data.params.max_delay,...
            data.params.doppler_bins, data.params.R);

        msg = sprintf('Object found:\n\nRange = %.2f km\nVelocity = %.2f m/s\n\nRemove?',...
                      r_km, v_ms);
        choice = questdlg(msg,'CLEAN','Yes','No','No');

        if strcmp(choice,'Yes')
            data.lastSurv = x_clean;
            
            [caf, delay_axis, doppler_axis] = CAF(data.ref,data.lastSurv,...
                data.params.fs, data.params.fc,...
                data.params.max_delay, data.params.doppler_bins, data.params.R,data.params.window_type);
            plotCAF(caf,delay_axis,doppler_axis,'CAF after CLEAN iteration');
            data.lastCAF = caf;
            data.delay_axis = delay_axis;
            data.doppler_axis = doppler_axis;
            axes.delay = delay_axis;
            axes.doppler = doppler_axis;
            addToHistory(caf,x_clean,data.params.window_type,'CLEAN',axes);
        end
        updateStatus();
    end
    
    function plotSpectrum()
        if ~isfield(data,'ref') && ~isfield(data,'surv')
            uialert(fig,'Choose signal','Błąd'); return;
        end
        
        
        plot_spectrum(data.ref,data.params.fs);
        title("Ref spectrum");

        plot_spectrum(data.lastSurv,data.params.fs);
        title("Surv spectrum");
        
        % choice = questdlg('Signal Spectrum','Choose Signal','ref','surv','ref');
        % 
        % if (strcmp(choice, 'ref'))
        %     if isfield(data,'ref')
        %         plot_spectrum(data.ref,data.params.fs);
        %         title("Ref spectrum");
        %     else
        %         uialert(fig,'Choose signal','Błąd'); return;
        %     end
        % elseif(strcmp(choice,'surv')) 
        %     if isfield(data,'lastSurv')
        %         plot_spectrum(data.lastSurv,data.params.fs);
        %         title("Surv spectrum");
        %     elseif isfield(data, 'surv')
        %         plot_spectrum(data.surv,data.params.fs);
        %         title("Surv spectrum");
        %     else
        %         uialert(fig,'Choose signal','Błąd'); return;
        %     end
        % else
        %     return;
        % end
    end
    
    function runXcorr()
        [corr, lags] = xcorr(data.lastSurv,data.ref,data.params.max_delay);
        corr = abs(corr)/max(abs(corr));
        D = 3e8 * lags./ data.params.fs;
        figure;
        plot(D/1000, mag2db(abs(corr)));
        title("Cross-correlation");
        fprintf("XCorr called\n");
    end

    function runPlotTime()
        if ~isfield(data,'ref') && ~isfield(data,'surv')
            uialert(fig,'Choose signal','Błąd'); return;
        end
          
        N = length(data.ref);
        time_ax = linspace(0,N/data.params.fs,N);

        figure;
        plot(time_ax,data.ref);
        title("Time ref");
        
        figure;
        plot(time_ax,data.lastSurv);
        title("Time surv");
    end
                    
                    % === History management ===
    function addToHistory(caf,surv_state,window_type,tag,axes)
        entry.caf = caf;
        entry.surv = surv_state;
        entry.window_type = window_type; 
        entry.mode = tag;
        entry.axes = axes;
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
        window_type = entry.window_type;
        axes = entry.axes;
        % ustaw jako bieżące
        data.lastCAF = caf;
        data.lastSurv = surv_state;
        data.lastWindow_type = window_type;
        plotCAF(caf,axes.delay,axes.doppler,'History: ' + string(mode));
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

    function rstHistory()
        data.history = {};
        data.histTitles = {};
        lstHistory.Items = data.histTitles;
        data.lastSurv = data.surv;
    end
    
                    % === Simulation helper ===
    function sim_sig = simulationGenSig()
        fprintf("\nSimulation function called\n");
    
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
        data.simulation.hist{data.simulation.params.add_echo_counter}.x_ref = sim_sig;
    end
    
    function addEcho()
        fprintf("\nAdd echo called\n");
        x_ref = simulationGenSig();
        

        params = data.simulation.params;
        fs = data.simulation.params.fs;
        range_m = data.simulation.params.Echo_position;
        velocity_ms = data.simulation.params.Echo_velocity;
        fc = data.simulation.params.fc;
        atten = data.simulation.params.Attenuation;
        dpi = data.simulation.params.DPI;
        %isClutter = data.simulation.params.Clutter;
        counter = data.simulation.params.add_echo_counter;
        
        
        % clutter(1).delay = 120;    % m
        % clutter(1).mag   = 0.25;   % -12 dB
        % clutter(2).delay = 600;    % m
        % clutter(2).mag   = 0.12;   % -19 dB
        % clutter(3).delay = 1800;   % m
        % clutter(3).mag   = 0.05;   % -26 dB
        % clutter(4).delay = 3000;   % m
        % clutter(4).mag   = 0.50;   % -6 dB

        
        N = 1000;
        clutter_range_m = 1+ rand(1,N)*30e3;
        clutter_range_m = sort(clutter_range_m);
        raw = 1./clutter_range_m;
        clutter_mag = raw / max(raw);
        data.simulation.params.clutter_mag = clutter_mag;
        data.simulation.params.clutter_range_m = clutter_range_m;
        % clutter_range_m = [120; 600; 1800; 3000];
        % clutter_mag_dB = [0.25; 0.12; 0.05; 0.5];

        clutter = create_clutter(clutter_range_m,clutter_mag);

        if(~(data.simulation.active.x_ref == 0))
            raw_signal = data.simulation.active.x_ref;
            x_surv_first = data.simulation.active.x_surv;
            signal_name = [data.simulation.active.name '_2'];
            [x_ref, x_surv] = simulate_target_ref_surv_signals(raw_signal,fs, ...
            range_m,velocity_ms,fc,atten,dpi,clutter);
            x_surv = x_surv .* x_surv_first;
            data.simulation.hist{counter}.name = signal_name;
            data.simulation.params.File_Name = signal_name;
        else
            raw_signal = data.simulation.hist{counter}.x_ref;       
            [x_ref, x_surv] = simulate_target_ref_surv_signals(raw_signal,fs, ...
            range_m,velocity_ms,fc,atten,dpi,clutter);
            data.simulation.hist{counter}.name = data.simulation.params.File_Name;
        end
        data.simulation.hist{counter}.x_ref = x_ref;
        data.simulation.hist{counter}.x_surv = x_surv;
        
        updateSimWorkspace();
        data.simulation.params.add_echo_counter = counter+1;
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
            ax.CLim = [mean(caf(:)), max(caf(:))];
        elseif chkFixed.Value
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
            lambda  = freq2wavelen(data.params.fc);
            T = length(data.ref)/data.params.fs;
            sz = size(data.lastCAF);
            lines{end+1} = sprintf('CAF size: %d x %d',sz(1),sz(2));
            data.log_lines = logTerminal(sprintf('Max power: %.2f dB',max(data.lastCAF(:))),'N',data.log_lines);
            data.log_lines = logTerminal(sprintf('Mean power: %.2f dB',mean(data.lastCAF(:))),'A',data.log_lines);
            lines{end+1} = sprintf('Distance resolution: %.1f m',3e8/data.params.fs);
            lines{end+1} = sprintf('Velocity resolution: %.3f m/s',lambda/T);
            lines{end+1} = sprintf('Signal duration: %.3f s',T);
            if isfield(data,'lastWindow_type')
                data.log_lines = logTerminal(sprintf('Used window: %s',data.lastWindow_type),'A',data.log_lines);
            else
               data.log_lines = logTerminal(sprintf('Used window: %s','Nan'),'A',data.log_lines); 
            end
        else
            lines{end+1} = 'CAF: not computed yet';
        end
        
        
        txtStatus.Value = lines;
    end

    line_counter = 0;
    sim_lines = {};
    y_start = 50;
    line_height = 30;
    gap = 5;

    function updateSimWorkspace()
        line_counter = line_counter + 1;
        
        y_pos = hSimStatus - y_start - (line_counter-1)*(line_height + gap);
    
        data.simulation.sim_lines{line_counter} = sim_workspace_line_create(pSimStatus, ...
            [0, y_pos, rightW-2*gap, line_height], ...
            data.simulation.params.File_Name, data.simulation.params, ...
            @onDeleteLine);
        
    
    end
    
    function onDeleteLine(signalName)
        % Find index of deleted line
        idx = find(cellfun(@(L) isfield(L,'signalName') && strcmp(L.signalName,signalName), sim_lines));
        if isempty(idx), return; end
    
        % Delete from list
        sim_lines(idx) = [];
        line_counter = numel(sim_lines);
    
        % Re-stack remaining lines
        for k = 1:numel(sim_lines)
            newY = hSimStatus - y_start - (k-1)*(line_height + gap);
            updateLinePosition(sim_lines{k}, newY);
        end
    end

    function updateLinePosition(lineStruct, newY)

        if isempty(lineStruct) || ~isstruct(lineStruct)
            return;
        end
        fns = fieldnames(lineStruct);
        for i = 1:numel(fns)
            h = lineStruct.(fns{i});
            if ishghandle(h)
                pos = h.Position;
                pos(2) = newY;
                h.Position = pos;
            end
        end
    end

    function refreshSimWorkspace(parent)
        delete(allchild(parent));
        data.simulation.hist = {};
        data.simulation.active.x_ref = 0;
        data.simulation.active.x_surv = 0;
        line_counter = 0;
        data.simulation.params.add_echo_counter= 1;
        data.simulation.sim_lines = {};
    end

    function setHistActive()
        for i=1:numel(data.simulation.hist)
            if(data.simulation.sim_lines{i}.isActive.Value)
                data.simulation.active = data.simulation.hist{i};
                return;
            end
        end
    end

    function s = safeStr(d,field)
        if isfield(d,field)
            s = d.(field); 
        else 
            s = '(n/a)';
        end
    end
    
    function lines = logTerminal(info,type,lines) 
        if (type == 'N')
            lines = {};
            lines{end+1} = info;
        else 
            lines{end+1} = info;
        end

        terminal.Value = lines;

    end
    % === CLim helpers ===
    function setCLimManual()
        ax.CLim = [editCmin.Value editCmax.Value];
        chkAuto.Value = false;
        chkFixed.Value = false;
    end
        
    sliderMin.ValueChangedFcn = @(~,~) updateCLimFromSliders();
    sliderMax.ValueChangedFcn = @(~,~) updateCLimFromSliders();
    chkAuto.ValueChangedFcn   = @(~,~) updateCAFScaling();
    chkFixed.ValueChangedFcn  = @(~,~) updateCAFScaling();
    
    function updateCAFScaling()
        if chkAuto.Value && isfield(data,'lastCAF')
            ax.CLim = [mean(data.lastCAF(:)) max(data.lastCAF(:))];
        elseif chkFixed.Value && isfield(data,'lastCAF')
            ax.CLim = [mean(data.lastCAF(:)) 0];
        end
    end
    
    function updateCLimFromSliders()
        ax.CLim = [sliderMin.Value sliderMax.Value];
        chkAuto.Value   = false;
        chkFixed.Value  = false;
    end

end
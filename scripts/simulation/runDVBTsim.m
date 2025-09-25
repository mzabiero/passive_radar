function runDVBTsim(outFile,fs,cpLen,duration,mode)
%RUNDVBTSIM Run DVB-T simulation and save signal to file.
%   runDVBTsim(outFile, fs, cpLen, duration, mode)
    
    modelname = 'commdvbt_file';
       
    if ~exist('commdvbt_file.slx', 'file')  
       sprintf("There is no simulation system");
       return;
    end
    
    load_system(modelname);
    simin = Simulink.SimulationInput(modelname);
    
    assignin('base','outFile',outFile);
    ofdmMod_Path = [modelname '/OFDM Modulator'];

    spect = [modelname '/Spectrum Scope'];
    spect_sc = get_param(spect,'ScopeConfiguration');
    spect_sc.OpenAtSimulationStart = true;
    
    setBlockParameter(simin,ofdmMod_Path,'CyclicPrefixLength',num2str(cpLen));
    set_param(modelname,'StopTime',num2str(duration));
    set_param(modelname,'FixedStep', num2str(1/fs));
    
    sim(modelname);

    close_system(modelname);

end
function [sig] = runDVBTsim(fs,cpLen,duration,mode)
%RUNDVBTSIM Run DVB-T simulation and save signal to file.
%   runDVBTsim(fs, cpLen, duration, mode)

    modelname = 'commdvbt_save_to_file';

    if ~exist([modelname '.slx'], 'file')
        fprintf("There is no simulation system: %s.slx\n", modelname);
        return;
    end

    load_system(modelname);

    % configure OFDM parameters
    ofdmPath = [modelname '/OFDM Modulator'];

    switch lower(mode)
        case '2k'
            set_param(ofdmPath,'FFTLength','2048');
        case '8k'
            set_param(ofdmPath,'FFTLength','8192');
        otherwise
            error("Unknown DVB-T mode: %s (use '2k','8k')", mode);
    end

    set_param(ofdmPath,'CyclicPrefixLength',num2str(cpLen));

    % configure simulation timing
    set_param(modelname,'StopTime',num2str(duration));
    set_param(modelname,'Solver','FixedStepDiscrete');
    set_param(modelname,'FixedStep','auto');
    set_param(modelname,'ReturnWorkspaceOutputs','on');

    % run simulation
    simOut = sim(modelname);

    % extract signal
    sig = simOut.simSignal; 


    close_system(modelname,0);
    fprintf("Simulated Signal generated\n")
end

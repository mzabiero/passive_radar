clear all;close all;clc;



if exist('commdvbt_file.slx','file')
    modelname = 'commdvbt_file';
    load_system(modelname)
else
    modelname = 'commdvbt';
    open_system(modelname);
end


% Define Simulink(R) blocks as variables
spect = [modelname '/Spectrum Scope'];
plot  = [modelname '/Delayed Scatter Plot/Enabled Scatter Plot/Discrete-Time Scatter Plot Scope'];
get_param(spect,'')
% Set visibility parameters for scopes
spect_sc = get_param(spect,'ScopeConfiguration');
spect_sc.OpenAtSimulationStart = false;
set_param( plot,  'OpenScopeAtSimStart', 'off' );

sim(modelname);

spect_sc.OpenAtSimulationStart = true;
sim(modelname);

% Set the simulation parameters for the next display:
spect_sc.OpenAtSimulationStart = false;
set_param( plot,  'OpenScopeAtSimStart', 'on'  );
close_system(spect); % Close scopes from previous simulation

sim(modelname);
close_system(modelname,0);

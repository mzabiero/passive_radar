function y = saveIQblock(u)
%#codegen
% u is the input signal from DVB-T modulator
% y is a passthrough (so the system is not broken)
    y = zeros(2304,1);
    % Call your external function to save
    persistent hasSaved;
    if isempty(hasSaved)
        % Use evalin to read GUI variable 'outFile' from workspace
        fname = evalin('base','outFile');
        save_comlex_binary(u, fname);
        hasSaved = true;   % only save once
    end

    y = u; % passthrough
end

function x_surv_clutter = addClutter(x_ref, x_surv, clutter, fs)
% addClutter - Adds stationary clutter echoes to the surveillance signal
%
%   x_surv_clutter = addClutter(x_ref, x_surv, clutter, fs)
%
%   Inputs:
%       x_ref   - reference complex baseband signal (Nx1)
%       x_surv  - current surveillance signal (Nx1)
%       clutter - struct array with fields:
%                   .delay [m]   - delay in meters for each clutter echo
%                   .mag   [0..1]- magnitude scaling for each echo
%       fs      - sampling rate [Hz]
%
%   Output:
%       x_surv_clutter - surveillance signal with added stationary clutter
%
%   Notes:
%       - Each clutter echo is a delayed and scaled version of x_ref.
%       - Delays beyond signal length are ignored.
%       - All clutter echoes are summed into x_surv.
%
%   Example:
%       clutter(1).delay = 300;   % meters
%       clutter(1).mag   = 0.3;
%       clutter(2).delay = 700;
%       clutter(2).mag   = 0.15;
%       x_surv_clutter = addClutter(x_ref, x_surv, clutter, fs);

    c = 3e8; % Speed of light [m/s]
    N = length(x_ref);
    x_surv_clutter = x_surv;

    if isempty(clutter)
        warning('addClutter:Empty', 'No clutter entries specified.');
        return;
    end

    for i = 1:numel(clutter)
        % Convert delay in meters to samples
        tau = clutter(i).range_m / c; 
        delaySamples = round(tau * fs);

        if delaySamples >= N
            warning('Clutter echo #%d delay exceeds signal length, skipping.', i);
            continue;
        end

        % Create delayed version of x_ref
        echo = [zeros(delaySamples,1); x_ref(1:end-delaySamples)];

        % Scale by magnitude
        echo = clutter(i).magnitude * echo;

        % Add to surveillance signal
        x_surv_clutter = x_surv_clutter + echo;
    end
end

function [x_ref, x_surv] = simulate_target_ref_surv_signals( ...
        data, fs, range_m, velocity_ms, fc, atten, dpi, clutter)

%   Simulate reference and surveillance signals with multiple moving targets,
%   optional DPI (direct path interference), and optional clutter.
%
%   Inputs:
%     data        - complex baseband input signal (reference)
%     fs          - sampling rate [Hz]
%     range_m     - vector of ranges for each target     [meters]
%     velocity_ms - vector of radial velocities          [m/s]
%     fc          - carrier frequency                    [Hz]
%     atten       - vector of attenuation factors (0..1) for targets
%     dpi         - 1 = add DPI, 0 = no DPI
%     clutter     - clutter structure, used with addClutter()
%
%   Outputs:
%     x_ref  - reference channel
%     x_surv - surveillance channel with all targets + DPI + clutter
%
%   Notes:
%     All vectors (range_m, velocity_ms, atten) must be same length,
%     each element corresponds to one target.

    c = 3e8;
    dpi_atten = 0.999;

    % -------------------------------
    % Input signal
    % -------------------------------
    x_ref = data(:);         % ensure column vector
    N = length(x_ref);

    % -------------------------------
    % Ensure all target vectors match
    % -------------------------------
    range_m     = range_m(:);
    velocity_ms = velocity_ms(:);
    atten       = atten(:);

    if ~(length(range_m) == length(velocity_ms) && length(velocity_ms) == length(atten))
        error('range_m, velocity_ms, and atten must all be the same length.');
    end

    numTargets = length(range_m);

    % -------------------------------
    % Initialize surveillance
    % -------------------------------
    x_surv = zeros(N,1);

    % -------------------------------
    % Add ALL target echoes
    % -------------------------------
    for k = 1:numTargets
        
        % --- Target parameters ---
        tau = range_m(k) / c;          % echo delay [seconds]
        delaySamples = round(tau * fs);
        
        if delaySamples >= N
            warning('Target %d echo delay exceeds signal length -> skipping.', k);
            continue;
        end
        
        % --- Doppler shift ---
        lambda = c / fc;
        fd = 2 * velocity_ms(k) / lambda;   % Doppler frequency
        
        % --- Create single target echo ---
        echo = [zeros(delaySamples,1); x_ref(1:end-delaySamples)];
        
        % Apply Doppler
        n = (0:length(echo)-1).';
        doppler_phase = exp(1j*2*pi*fd*n/fs);
        echo = atten(k) * echo .* doppler_phase;
        
        % Add to surveillance
        x_surv = x_surv + echo;
    end

    % -------------------------------
    % Add DPI (direct path interference)
    % -------------------------------
    if dpi == 1
        x_surv = add_dpi(dpi_atten, x_ref, x_surv);
    end

    % -------------------------------
    % Add clutter echoes
    % -------------------------------
    if isstruct(clutter)
        x_surv = addClutter(x_ref, x_surv, clutter, fs);
    end

end

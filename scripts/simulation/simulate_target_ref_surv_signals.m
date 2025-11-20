function [x_ref, x_surv] = simulate_target_ref_surv_signals(data, fs, range_m, velocity_ms, fc,...
    atten,dpi,clutter)
%   [x_ref, x_surv] = create_ref_surv_from_file(data, fs, range_m, velocity_ms, fc, atten,dpi, clutter)
%
%   data        - input data that's processed to create ref and surv
%   signals
%   fs          - sampling rate in Hz
%   range_m     - echo delay in meters
%   velocity_ms - radial velocity of target in m/s
%   fc          - carrier frequency in Hz (e.g. 650e6 for DVB-T channel)
%   atten       - attenuation factor for echo (0..1)
%   dpi         - determines if direct path interferance is added to the
%  surv signal
%   clutter     - determines if clutter reflections and echoes are added
%  to the surv signal
%
%   Outputs:
%     x_ref - reference signal (same as input)
%     x_surv - surveillance signal (delayed, Doppler-shifted echo)

    c = 3e8;  % speed of light [m/s]
    dpi_atten = 0.999;

    % --- read input file ---
    % data = read_complex_binary(infile);
    % fid = fopen(infile,'rb');
    % raw = fread(fid,'float32');
    % fclose(fid);
    % 
    % raw = reshape(raw,2,[]);
    % data = complex(raw(1,:), raw(2,:)).';   % column vector

    N = length(data);

    
    % --- reference signal ---
    x_ref = data;

    % --- convert range (m) to delay in samples ---
    tau = range_m / c;  % delay in seconds
    delaySamples = round(tau * fs);

    % --- Doppler shift from velocity (radial) ---
    lambda = c / fc;
    dopplerHz = 2 * velocity_ms / lambda;

    % --- create surveillance signal ---
    if delaySamples >= N
        error('Delay too large: exceeds signal length!');
    end

    % Apply delay
    x_surv = [zeros(delaySamples,1); data(1:end-delaySamples)];

    % Apply Doppler
    n = (0:length(x_surv)-1).';
    doppler_phase = exp(1j*2*pi*dopplerHz*n/fs);
    x_surv = atten * x_surv .* doppler_phase;
    % Add DPI
    if(dpi == 1)
        x_surv = add_dpi(dpi_atten,x_ref,x_surv);
    end
    % Add Clutter echoes
    % if(clutter.isClutter == 1)
    %     x_surv = add_clutter(x_surv,fs,clutter);
    % end
    x_surv = addClutter(x_ref,x_surv,clutter,fs);
    
end

function [x_ref, x_surv] = simulate_target_ref_surv_signals(infile, fs, range_m, velocity_ms, fc, atten)
%CREATE_REF_SURV_FROM_FILE  Generate reference and surveillance signals
%   [x_ref, x_surv] = create_ref_surv_from_file(infile, fs, range_m, velocity_ms, fc, atten)
%
%   infile      - input .dat file with IQ data (float32 interleaved)
%   fs          - sampling rate in Hz
%   range_m     - echo delay in meters
%   velocity_ms - radial velocity of target in m/s
%   fc          - carrier frequency in Hz (e.g. 650e6 for DVB-T channel)
%   atten       - attenuation factor for echo (0..1)
%
%   Outputs:
%     x_ref - reference signal (same as input)
%     x_surv - surveillance signal (delayed, Doppler-shifted echo)

    c = 3e8;  % speed of light [m/s]

    % --- read input file ---
    data = read_complex_binary(infile);
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
    
    sim_save_del(infile,x_ref,x_surv); 

    
end

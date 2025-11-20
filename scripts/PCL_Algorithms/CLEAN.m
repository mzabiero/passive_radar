function [x_surv_clean, bistatic_range_km, bistatic_velocity] = CLEAN(...
        caf_matrix, x_ref, x_surv, fs, fc, max_delay, doppler_bins, R)
% CLEAN - remove the strongest CAF peak echo from surveillance signal
% 
% Inputs:
%   caf_matrix    : cross-ambiguity function matrix (delay x doppler)
%   x_ref, x_surv : reference and surveillance complex baseband vectors (Nx1)
%   fs            : sampling frequency (Hz) used to form CAF
%   fc            : carrier frequency (Hz)
%   max_delay     : number of delay bins in CAF (usually rows of caf_matrix)
%   doppler_bins  : number of doppler bins in CAF (usually columns of caf_matrix)
%   R             : optional decimation/processing factor used when computing CAF.
%                   If CAF used a decimated fs (fs_dec = fs/R), pass that R.
%
% Outputs:
%   x_surv_clean       : surveillance signal with strongest estimated echo removed
%   bistatic_range_km  : bistatic range (km) corresponding to detected delay bin
%   bistatic_velocity  : estimated velocity (m/s) corresponding to detected doppler bin

    if nargin < 8 || isempty(R)
        R = 1; % default: no decimation
    end

    c = 3e8;
    lambda = freq2wavelen(fc,c);

    % 1) finding strongest peak
    [max_val, linear_idx] = max(caf_matrix(:));
    [delay_idx, doppler_idx] = ind2sub(size(caf_matrix), linear_idx);
    
    max_val

    if size(caf_matrix,1) ~= max_delay
        % adapt max_delay to the actual number of rows
        max_delay = size(caf_matrix,1);
    end

    % delay_samples = 0:(max_delay-1);          % sample delays corresponding to CAF rows
    % delay_samp = delay_samples(delay_idx);    % integer sample delay (0-based)

    % 3) compute bistatic range (total path length) in km
    %    Range = c * tau, tau = delay_samp / fs -> path length in meters, /1000 -> km
    bistatic_range_km = (c * ( (delay_idx-1) / fs)) / 1000;

    % 4) build Doppler (frequency) axis taking into account any decimation factor R
    fs_dec = fs / R;  % effective sampling rate used when computing CAF
    % center the doppler bins around zero:
    fd_axis = ((0:doppler_bins-1) - floor(doppler_bins/2)) * (fs_dec / doppler_bins); % Hz
    % convert Doppler frequency to velocity (monostatic approx)
    vel_axis = fd_axis * (lambda / 2);  % m/s
    % selected doppler/velocity
    fd = fd_axis(doppler_idx);
    bistatic_velocity = vel_axis(doppler_idx);

    % 5) build echo model: delayed reference, modulated by Doppler
    N = length(x_ref);
    t_dec = (0:N-1).' / fs_dec;   % time vector for the (possibly decimated) rate

    % Create delayed signal with front padding.
    x_surv_clean = [zeros(delay_idx,1); x_ref(1:end-delay_idx)];

    % Apply Doppler
    n = (0:length(x_surv)-1).';
    doppler_phase = exp(1j*2*pi*fd*n/fs);
    echo_model =x_surv_clean .* doppler_phase; 

    
    ampl_est = ((x_surv') * echo_model) / ((echo_model') * echo_model);
    
    
    % 7) subtract estimated echo
    echo_est = ampl_est * echo_model;
    x_surv_clean = x_surv - echo_est;

    % (optional) debug print
    % fprintf('CLEAN: peak mag=%.3g at delay=%d doppler=%d, fd=%.2f Hz, v=%.2f m/s, alpha=%.3g\n', ...
    %         abs(caf_matrix(linear_idx)), delay_idx, doppler_idx, fd, bistatic_velocity, alpha_hat);

end

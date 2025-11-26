function [caf_matrix_dB, delay_axis_km, doppler_axis_ms] = ...
    CAF(x_ref, x_surv, fs, fc, max_delay, doppler_bins, R, window_type, back_delay)

disp("CAF running...");
c = 3e8;
lambda = c / fc;

% Window selection
if nargin < 8 && window_type
    is_window = false;
    disp("No windowing applied");
else
    is_window = true;
    window_len = length(x_ref) / R;
    window_ym = window_CAF(window_len, window_type);
end

% Build delay index vector: negative → positive
delays = (-back_delay) : (max_delay-1);
num_delays = length(delays);

% Allocate CAF matrix
caf_matrix = zeros(num_delays, doppler_bins);

% Parallel config
cpu_count = 10;
default_worker_number = 10;
worker_number = min(default_worker_number, cpu_count);

% Main CAF loop
parfor (i = 1:num_delays, worker_number)

    d = delays(i);     % current delay (negative or positive)

    % Apply delay
    if d >= 0
        % forward delay: surv .* conj(shifted ref)
        ym = x_surv .* conj([zeros(d,1); x_ref(1:end-d)]);
    else
        % backward delay: surv .* conj(ref shifted backwards)
        dm = abs(d);
        ym = x_surv .* conj([x_ref(dm+1:end); zeros(dm,1)]);
    end

    % Decimation
    ym_dec = decimate(ym, R);

    % Windowing
    if is_window
        ym_dec = ym_dec .* window_ym;
    end

    % Doppler FFT
    ym_fft = fftshift(fft(ym_dec, doppler_bins));
    caf_matrix(i, :) = abs(ym_fft);

end

% Convert to dB
caf_matrix_dB = mag2db(caf_matrix + eps);

% Axis computations
delay_samples = delays;
delay_axis_km = (c/fs) * delay_samples / 1000;

f_dec = fs / R;
k = (-doppler_bins/2):(doppler_bins/2 - 1);
fd = (f_dec/doppler_bins) * k;
doppler_axis_ms = (lambda/2) .* fd;

end

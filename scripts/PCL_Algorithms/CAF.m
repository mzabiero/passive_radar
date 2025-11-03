function [caf_matrix_dB, delay_axis_km,doppler_axis_ms] = ...
    CAF(x_ref,x_surv,fs,fc,max_delay,doppler_bins, R,window_type)

    disp("CAF running...");
    c = 3e8;
    lambda = c/fc;

    if nargin < 8 || isempty(window_type)
        is_window = 0;
        disp("No windowing applied");
    else
        is_window = 1;
        window_len = length(x_ref) / R;
        window_ym = window_CAF(window_len, window_type);
    end
    
    
    
    caf_matrix = zeros(max_delay, doppler_bins);       
    cpu_count = 10; % make it being taken from system info
    default_worker_number = 10;
    worker_number = min(default_worker_number, cpu_count);
    
    if(is_window)
        parfor (m = 0:max_delay-1, worker_number)
            ym = x_surv .* conj([zeros(m,1); x_ref(1:end-m)]);
            ym_dec = decimate(ym,R);
            ym_dec = ym_dec.*window_ym; 
            ym_fft = fftshift(fft(ym_dec,doppler_bins));
            caf_matrix(m+1,:) = abs(ym_fft);   
        end
    else
        parfor (m = 0:max_delay-1, worker_number)
            ym = x_surv .* conj([zeros(m,1); x_ref(1:end-m)]);
            ym_dec = decimate(ym,R);
            ym_fft = fftshift(fft(ym_dec,doppler_bins));
            caf_matrix(m+1,:) = abs(ym_fft);   
        end
    end
    %caf_matrix_norm = caf_matrix / max(caf_matrix(:));  % normalizacja
    caf_matrix_dB = mag2db(abs(caf_matrix));
    max(caf_matrix_dB(:))
    
    %delay_axis = linspace(-max_delay/2, max_delay/2, doppler_bins);
    %delay_axis = (0:max_delay-1) * 3e8 / (fs * 1e3);  % [km]
    delay_samples = 0:max_delay-1;
    delay_axis_km = (c/fs)*delay_samples/1000;
    f_dec = fs/R;
    k = (-doppler_bins/2):(doppler_bins/2-1);
    fd = (f_dec/doppler_bins)*k;
    doppler_axis_ms =  (lambda/2).*fd;


    % Wybranie interesującego zakresu Dopplera
    % doppler_limit = 300;  % m/s
    % mask = abs(doppler_axis_ms) <= doppler_limit;
    % 
    % doppler_axis_ms = doppler_axis_ms(mask);
    % caf_matrix = caf_matrix_dB(:,mask);
    % disp(".")
    % Rysowanie
    % figure;
    % imagesc(doppler_axis_ms, delay_axis_km, caf_matrix_dB);
    % clim([mean(caf_matrix_dB(:)), 0])
    % %xlim([-doppler_limit doppler_limit]);
    % xlabel('Bistatic velocity, V (m/s)');
    % ylabel('Bistatic range, R (km)');
    % colormap jet; colorbar;

end
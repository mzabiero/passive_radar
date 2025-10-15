function [caf_matrix_dB, delay_axis_km,doppler_axis_ms] = ...
    CAF(x_ref,x_surv,fs,fc,max_delay,doppler_bins, R)


    disp("CAF running...");
    c = 3e8;
    lambda = c/fc;

    caf_matrix = zeros(max_delay, doppler_bins);       
    cpu_count = 10; % make it being taken from system info
    default_worker_number = 10;
    worker_number = min(default_worker_number, cpu_count);

    parfor (m = 0:max_delay-1,worker_number)
        %ym = x_surv .* conj([zeros(m,1); x_ref(1:end-m)]);
        ym = x_surv .* conj(circshift(x_ref,m));
        %ym_lpf = lowpass(ym,1/R);
        %ym_lpf = lowpass(ym, fs / (2 * R), fs);
        %ym_dec = ym_lpf(1:R:end);
        ym_dec = decimate(ym,R);
        %ym_dec = decimate(ym_dec,10); 
        ym_fft = fftshift(fft(ym_dec,doppler_bins));
        caf_matrix(m+1,:) = abs(ym_fft);   
    end

    %caf_matrix_norm = caf_matrix / max(caf_matrix(:));  % normalizacja
    caf_matrix_dB = mag2db(abs(caf_matrix));

    
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
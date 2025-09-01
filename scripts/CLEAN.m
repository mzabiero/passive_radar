function [x_surv_clean, bistatic_range_km, bistatic_velocity] = CLEAN(caf_matrix,...
    x_ref, x_surv, fs, fc, max_delay, doppler_bins, R)

    disp("CLEAN running...");
    c = 3e8;
    lambda = c/fc;

    % znajdź maksimum
    [max_val, linear_idx] = max(caf_matrix(:));
    [delay_idx, doppler_idx] = ind2sub(size(caf_matrix),linear_idx);
    max_val
    % osie
    delay_samples = 0:max_delay-1;
    delay_axis_km = (c/fs) * delay_samples / 1000;

    f_dec = fs/R;
    k = (-doppler_bins/2):(doppler_bins/2-1);
    fd_axis = (f_dec/doppler_bins) * k;       % Doppler frequency [Hz]
    vel_axis = (lambda/2) * fd_axis;          % bistatic velocity [m/s]
    % doppler_limit = 300;  % m/s
    % mask = abs(vel_axis) <= doppler_limit;
    % 
    %vel_axis = vel_axis(mask);
   
    % parametry celu
    bistatic_range_km = delay_axis_km(delay_idx);
    bistatic_velocity = vel_axis(doppler_idx);

    % model echa
    delay_samp = delay_idx-1;
    f_d = fd_axis(doppler_idx);               % Doppler freq [Hz]
    n = (0:length(x_ref)-1).';
    echo_model = circshift(x_ref, delay_samp) .* exp(1j*2*pi*f_d*n/fs);

    % estymacja amplitudy
    alpha_hat = (x_surv' * echo_model) / (echo_model' * echo_model);

    % odjęcie echa
    echo_est = alpha_hat * echo_model;
    x_surv_clean = x_surv - echo_est;
end

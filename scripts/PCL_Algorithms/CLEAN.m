function [x_surv_clean, bistatic_range_km, bistatic_velocity] = CLEAN(...
        caf_matrix, x_ref, x_surv, fs, fc, max_delay, doppler_bins, R)
% CLEAN_FINAL - Usuwa echo z wykorzystaniem interpolacji i ułamkowego opóźnienia.
%
% WAŻNE: caf_matrix musi być ZESPOLONA (surowa), a nie w dB!

    if nargin < 8 || isempty(R), R = 1; end
    
    c = 3e8; 
    lambda = c / fc;
    fs_dec = fs / R; % Częstotliwość próbkowania CAF
    
    x_ref = x_ref(:);
    x_surv = x_surv(:);
    N = length(x_ref);

    % --- 1. Znalezienie piku na module (Linear Magnitude) ---
    % Używamy abs(), aby znaleźć energię. Max na liczbach zespolonych nie ma sensu.
    % Jeśli Twoja caf_matrix jest w dB, to TU JEST BŁĄD w wywołaniu funkcji.
    mag_matrix = abs(caf_matrix);
    [~, linear_idx] = max(mag_matrix(:));
    [delay_idx_int, doppler_idx_int] = ind2sub(size(mag_matrix), linear_idx);

    % --- 2. Interpolacja 3-Punktowa (Paraboliczna) ---
    % Znajdujemy ułamkowe przesunięcia delty
    delta_doppler = interpolate_3point(mag_matrix, delay_idx_int, doppler_idx_int, 2);
    delta_delay   = interpolate_3point(mag_matrix, delay_idx_int, doppler_idx_int, 1);
    
    % Precyzyjne indeksy (float)
    doppler_idx_float = doppler_idx_int + delta_doppler;
    delay_idx_float   = delay_idx_int   + delta_delay;

    % --- 3. Fizyka (Obliczenie parametrów) ---
    
    % A. Delay: Indeks 1 w CAF to 0 opóźnienia. Mnożymy przez R (decymacja)
    % delay_idx_float jest w próbkach zdecymowanych (fs_dec)
    tau_samples_dec = delay_idx_float - 1; 
    tau_samples_full = tau_samples_dec ; % Opóźnienie w pełnym fs
    
    bistatic_range_km = (c * (tau_samples_full / fs)) / 1000;

    % B. Doppler: Wyśrodkowanie osi częstotliwości
    freq_res = fs_dec / doppler_bins;
    % Zakładamy, że DC (0Hz) jest w binie floor(N/2)+1
    bin_offset = floor(doppler_bins/2) + 1;
    fd = (doppler_idx_float - bin_offset) * freq_res;
    
    bistatic_velocity = fd * (lambda / 2);

    % --- 4. Rekonstrukcja z Ułamkowym Opóźnieniem (Sinc Interpolation) ---
    % Zwykłe round() powoduje błędy. Musimy użyć filtra sinc, aby przesunąć
    % sygnał o ułamek próbki (np. 10.4 próbki).
    
    d = tau_samples_full;
    d_int = floor(d);
    d_frac = d - d_int;
    
    % Generowanie filtra Sinc przesuniętego o d_frac
    L_kernel = 30; % Długość połówkowa filtra (im więcej tym dokładniej, ale wolniej)
    n_k = -L_kernel:L_kernel;
    h = sinc(n_k - d_frac) .* hann(2*L_kernel+1)'; % Windowed Sinc
    h = h / sum(h); % Normalizacja energii

    % Przesunięcie całkowite (Integer Delay)
    if d_int >= N
        x_ref_final = zeros(N,1);
    else
        % Splot 'same' centruje wynik, więc musimy uważać na indeksy.
        % Dla uproszczenia w CLEAN przyjmuje się często podejście:
        % 1. Shift integer. 2. Filter fractional.        
        % Prostsze podejście (Shift then Filter):
        temp = [zeros(d_int,1); x_ref(1:max(1, end-d_int))];
        % Teraz delikatne przesunięcie o d_frac filtrem
        x_ref_final = conv(temp, h, 'same'); 
        
        % Korekta długości po splocie
        x_ref_final = x_ref_final(1:N);
    end

    % --- 5. Rekonstrukcja Dopplera i LS ---
    t_vec = (0:N-1).' / fs;
    doppler_phasor = exp(1i * 2 * pi * fd * t_vec);
    
    echo_model = x_ref_final .* doppler_phasor;
    
    % Least Squares (Dopasowanie amplitudy i fazy)
    % alpha = (h' * y) / (h' * h)
    denom = echo_model' * echo_model;
    if abs(denom) < 1e-12
        alpha = 0;
    else
        alpha = (echo_model' * x_surv) / denom;
    end
    
    % --- 6. Odejmowanie ---
    echo_estimated = alpha * echo_model;
    x_surv_clean = x_surv - echo_estimated;

    % --- Debug Plot ---
    % Rysujemy interpolację dla pewności
    figure; clf;
    tiledlayout(2,1);
    nexttile; interpolate_3point(mag_matrix, delay_idx_int, doppler_idx_int, 2, true); title('Doppler Interp');
    nexttile; interpolate_3point(mag_matrix, delay_idx_int, doppler_idx_int, 1, true); title('Delay Interp');

    fprintf('CLEAN Final: Bin(%.2f, %.2f) -> R=%.2f m, V=%.2f m/s\n', ...
        delay_idx_float, doppler_idx_float, bistatic_range_km*1000, bistatic_velocity);
end

% --- Pomocnicza: Interpolacja ---
function delta = interpolate_3point(M, r, c, dim, do_plot)
    if nargin < 5, do_plot = false; end
    [rows, cols] = size(M);
    
    if dim == 1 % Delay
        if r <= 1 || r >= rows, delta=0; return; end
        vals = M(r-1 : r+1, c);
        center_idx = r;
    else % Doppler
        if c <= 1 || c >= cols, delta=0; return; end
        vals = M(r, c-1 : c+1).';
        center_idx = c;
    end
    
    y1 = vals(1); y2 = vals(2); y3 = vals(3);
    denom = 2 * (y1 - 2*y2 + y3);
    
    if abs(denom) < 1e-10, delta = 0;
    else, delta = (y1 - y3) / denom; end
    
    if abs(delta) > 0.6, delta = 0; end
    
    if do_plot
        x = -1:1;
        plot(x, vals, 'yo-'); hold on;
        x_fine = linspace(-1,1,50);
        a = (y1 - 2*y2 + y3)/2; b = (y3 - y1)/2; c_par = y2;
        plot(x_fine, a*x_fine.^2 + b*x_fine + c_par, 'r--');
        plot(delta, a*delta^2 + b*delta + c_par, 'g*');
        hold off; grid on;
    end
end
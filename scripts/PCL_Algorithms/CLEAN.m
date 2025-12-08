function [x_surv_clean, bistatic_range_km, bistatic_velocity] = CLEAN(...
        caf_matrix, x_ref, x_surv, fs, fc, max_delay, doppler_bins, R)
% CLEAN - Usuwa echo używając ścisłej interpolacji 3-punktowej.

    if nargin < 8 || isempty(R)
        R = 1; 
    end
    
    c = 3e8; 
    lambda = c / fc;
    
    x_ref = x_ref(:);
    x_surv = x_surv(:);
    N = length(x_ref);

    % --- 1. Znalezienie zgrubnego piku (na siatce) ---
    % Używamy abs() - amplituda liniowa jest lepsza do interpolacji 
    % paraboli blisko szczytu niż logarytmiczna (dB), choć obie działają.
    mag_matrix = caf_matrix;
    [~, linear_idx] = max(caf_matrix(:));
    [delay_idx_int, doppler_idx_int] = ind2sub(size(mag_matrix), linear_idx);

    % --- 2. Interpolacja 3-Punktowa z wizualizacją ---
    % Tworzymy okno debugowania tylko dla pierwszego wywołania lub zawsze (opcjonalnie)
    figure('Name', 'CLEAN Exact Interpolation', 'NumberTitle', 'off');
    tiledlayout(2,1);

    % A. Interpolacja Dopplera (oś X)
    nexttile;
    delta_doppler = interpolate_3point(mag_matrix, delay_idx_int, doppler_idx_int, 2, 'Doppler');
    doppler_idx_float = doppler_idx_int + delta_doppler;
    
    % B. Interpolacja Opóźnienia (oś Y)
    nexttile;
    delta_delay = interpolate_3point(mag_matrix, delay_idx_int, doppler_idx_int, 1, 'Delay');
    delay_idx_float = delay_idx_int + delta_delay;

    % --- 3. Obliczenie fizycznych parametrów ---
    
    % Opóźnienie (pamiętamy o decymacji R i że indeks 1 to 0s)
    delay_axis_samp = 0:max_delay-1;
    delay_axis_m = c .* delay_axis_samp ./ fs;
    % bistatic_range_m = delay_axis_m(delay_idx_int - 1);
    tau_samples_caf = (delay_idx_float - 1); 
    tau_samples_high_res = tau_samples_caf; 
    
    bistatic_range_km = (c * (tau_samples_high_res / fs)) / 1000;

    % Doppler
    fs_caf = fs / R; 
    freq_resolution = fs_caf / doppler_bins;
    
    % Przeliczenie indeksu na Hz (zakładając zero w połowie - fftshift)
    % Używamy floor(doppler_bins/2) + 1 jako środka (DC)
    fd = (doppler_idx_float - (floor(doppler_bins/2) + 1)) * fre;
    
    bistatic_velocity = fd * (lambda / 2);

    % --- 4. Rekonstrukcja sygnału ---
    
    % A. Opóźnienie (Round do najbliższej próbki)
    tau_int = round(tau_samples_caf);
    
    if tau_int >= N
         x_ref_delayed = zeros(N, 1);
    else
         x_ref_delayed = [zeros(tau_int, 1); x_ref(1:end-tau_int)];
    end

    % B. Doppler (Precyzyjna faza)
    t_vec = (0:N-1).' / fs; 
    doppler_phasor = exp(1i * 2 * pi * fd * t_vec);
    
    echo_model = x_ref_delayed .* doppler_phasor;

    % --- 5. Least Squares (Amplituda zespolona) ---
    numerator = x_surv' * echo_model;       
    denominator = echo_model' * echo_model; 
    
    if denominator < 1e-10
        alpha_est = 0;
    else
        alpha_est = numerator / denominator;
    end
    
    % --- 6. Odejmowanie ---
    echo_estimated = alpha_est * echo_model;
    x_surv_clean = x_surv - echo_estimated;
    
    fprintf('CLEAN Exact: Bin(%.2f, %.2f) -> R=%.2f m, V=%.2f m/s\n', ...
        delay_idx_float, doppler_idx_float, bistatic_range_km*1000, bistatic_velocity);
end

% --- Funkcja pomocnicza: Interpolacja 3-punktowa ---
function delta = interpolate_3point(M, r, c, dim, label)
    % M - macierz amplitud (liniowa, abs)
    % r, c - współrzędne piku całkowitego
    % dim - 1 (Delay/Wiersze), 2 (Doppler/Kolumny)
    
    [rows, cols] = size(M);
    
    % Pobranie 3 punktów: y1 (lewy/górny), y2 (środek), y3 (prawy/dolny)
    if dim == 1 % Delay (Wiersze)
        if r <= 1 || r >= rows
            delta = 0; title([label ': Edge case']); return; 
        end
        vals = M(r-1 : r+1, c);
        x_idx = -1:1;
        center_idx = r;
    else % Doppler (Kolumny)
        if c <= 1 || c >= cols
            delta = 0; title([label ': Edge case']); return; 
        end
        vals = M(r, c-1 : c+1).'; % Transpozycja na wektor pionowy
        x_idx = -1:1;
        center_idx = c;
    end
    
    y1 = vals(1);
    y2 = vals(2); % To jest nasz pik z max()
    y3 = vals(3);
    
    % Wzór na wierzchołek paraboli przechodzącej przez (-1,y1), (0,y2), (1,y3)
    % delta = (y1 - y3) / (2 * (y1 - 2*y2 + y3))
    denom = 2 * (y1 - 2*y2 + y3);
    
    if abs(denom) < 1e-10
        delta = 0; % Płasko lub linia prosta
    else
        delta = (y1 - y3) / denom;
    end
    
    % Zabezpieczenie (delta powinna być w [-0.5, 0.5] jeśli y2 jest max)
    if abs(delta) > 0.6
        delta = 0; 
    end

    % --- Rysowanie Debug ---
    % Punkty pomiarowe
    plot(x_idx + center_idx, vals, 'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Samples'); 
    hold on; grid on;
    
    % Analityczna parabola dla wizualizacji
    % y = a*x^2 + b*x + c
    % c = y2
    % b = (y3 - y1) / 2
    % a = (y1 - 2*y2 + y3) / 2
    param_c = y2;
    param_b = (y3 - y1) / 2;
    param_a = (y1 - 2*y2 + y3) / 2;
    
    x_fine = linspace(-1.5, 1.5, 50);
    y_fine = param_a * x_fine.^2 + param_b * x_fine + param_c;
    
    plot(x_fine + center_idx, y_fine, 'r--', 'DisplayName', 'Parabola fit');
    
    % Zaznaczenie wyliczonego piku
    peak_val = param_a * delta^2 + param_b * delta + param_c;
    plot(delta + center_idx, peak_val, 'g*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Interp. Peak');
    
    title(sprintf('%s: Center %d, Delta %.4f', label, center_idx, delta));
    xlabel('Index'); ylabel('Amplitude (Lin)');
    legend('Location','best');
    hold off;
end
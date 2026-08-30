function [cafLin, rAx, vAx] = calcCafBatched(ref, surv, fs, fc, minRange, maxRange, minVel, maxVel)
    c = 3e8;
    lambda = c / fc;
    N = min(length(ref), length(surv));
    
    max_R = max(abs(maxRange), abs(minRange));
    Q = ceil((max_R * 1000 / c) * fs) + 1;
    Q = max(Q, 64);
    
    P = floor(N / Q);
    if P < 1
        error('Sygnal zbyt krotki dla zadanego promienia MaxRange.');
    end
    
    ref_2D = reshape(ref(1:Q*P), Q, P);
    surv_2D = reshape(surv(1:Q*P), Q, P);
    
    N_fast = 2 * Q;
    winRange = ifftshift(hann(N_fast));
    winDoppler = hann(P)';
    
    R_f = fft(ref_2D, N_fast, 1);
    S_f = fft(surv_2D, N_fast, 1);
    
    cross_spec = (S_f .* conj(R_f)) .* winRange;
    corr_fast = ifft(cross_spec, [], 1);
    corr_fast_win = corr_fast .* winDoppler;
    
    caf_matrix = fftshift(fftshift(fft(corr_fast_win, [], 2), 2), 1);
    
    tau_full = linspace(-N_fast/2, N_fast/2 - 1, N_fast)' / fs;
    range_full = (tau_full * c) / 1000;
    
    F_prf = fs / Q;
    doppler_full = linspace(-F_prf/2, F_prf/2, P);
    vel_full = doppler_full * (lambda / 2);
    
    rangeIdx = range_full >= minRange & range_full <= maxRange;
    velIdx = vel_full >= minVel & vel_full <= maxVel;
    
    cafLin = caf_matrix(rangeIdx, velIdx);
    rAx = range_full(rangeIdx);
    vAx = vel_full(velIdx);
end
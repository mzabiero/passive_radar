function [cafLin, rAx, vAx] = calcCafDirect(ref, surv, fs, fc, minRange, maxRange, minVel, maxVel)
    c = 3e8;
    lambda = c / fc;
    N = min(length(ref), length(surv));
    
    minDelay = floor((minRange * 1000 / c) * fs);
    maxDelay = ceil((maxRange * 1000 / c) * fs);
    delays = minDelay:maxDelay;
    numDelays = length(delays);
    
    maxV = max(abs(maxVel), abs(minVel));
    if maxV == 0
        maxV = 1;
    end
    maxDoppler = maxV / (lambda / 2);
    
    R = max(1, floor(fs / (2 * maxDoppler)));
    
    dummy_dec = decimate(zeros(N, 1), R);
    N_dec = length(dummy_dec);
    dopplerBins = 2^nextpow2(N_dec);
    
    caf_matrix = zeros(numDelays, dopplerBins);
    window_ym = hann(N_dec);
    
    parfor i = 1:numDelays
        d = delays(i);
        if d >= 0
            ym = surv .* conj([zeros(d,1); ref(1:end-d)]);
        else
            dm = abs(d);
            ym = surv .* conj([ref(dm+1:end); zeros(dm,1)]);
        end
        
        ym_dec = decimate(ym, R);
        ym_dec = ym_dec .* window_ym;
        caf_matrix(i, :) = fftshift(fft(ym_dec, dopplerBins));
    end
    
    f_dec = fs / R;
    k = (-dopplerBins/2):(dopplerBins/2 - 1);
    doppler_full = (f_dec / dopplerBins) * k;
    vel_full = doppler_full * (lambda / 2);
    
    velIdx = vel_full >= minVel & vel_full <= maxVel;
    
    cafLin = caf_matrix(:, velIdx);
    rAx = (c / fs) * delays' / 1000;
    vAx = vel_full(velIdx);
end
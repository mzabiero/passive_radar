function [surv_clean, r_km, v_ms] = CLEAN(cafLin, rAx, vAx, ref, surv, fs, fc, rIdx, vIdx, useInterpolate)
    c = 3e8;
    lambda = c / fc;
    if nargin < 10 || isempty(useInterpolate)
        useInterpolate = false;
    end
    if nargin < 9 || isempty(rIdx) || isempty(vIdx)
        [~, linearIdx] = max(abs(cafLin), [], 'all');
        [rIdx, vIdx] = ind2sub(size(cafLin), linearIdx);
    end

    deltaR = 0;
    deltaV = 0;

    if useInterpolate
        [rows, cols] = size(cafLin);
        
        if rIdx > 1 && rIdx < rows
            valsR = abs(cafLin(rIdx-1:rIdx+1, vIdx));
            denomR = 2 * (valsR(1) - 2*valsR(2) + valsR(3));
            if abs(denomR) > 1e-10
                deltaR = (valsR(1) - valsR(3)) / denomR;
                if abs(deltaR) > 1.0, deltaR = 0; end
            end
        end
        
        if vIdx > 1 && vIdx < cols
            valsV = abs(cafLin(rIdx, vIdx-1:vIdx+1)).';
            denomV = 2 * (valsV(1) - 2*valsV(2) + valsV(3));
            if abs(denomV) > 1e-10
                deltaV = (valsV(1) - valsV(3)) / denomV;
                if abs(deltaV) > 1.0, deltaV = 0; end
            end
        end
    end

    dr = 0;
    dv = 0;
    if length(rAx) > 1
        dr = rAx(2) - rAx(1);
    end
    if length(vAx) > 1
        dv = vAx(2) - vAx(1);
    end

    r_km = rAx(rIdx) + deltaR * dr;
    v_ms = vAx(vIdx) + deltaV * dv;

    c = 3e8;
    tau = (r_km * 1000) / c;
    fd = v_ms / lambda;

    N = length(ref);
    padN = 2 * N; 
    
    df = fs / padN;
    fAxis = (0:padN-1)' * df;
    halfN = floor(padN/2);
    fAxis(halfN+2:end) = fAxis(halfN+2:end) - fs;

    ref_padded = zeros(padN, 1);
    ref_padded(1:N) = ref(:);

    Ref_f = fft(ref_padded);
    Shifted_f = Ref_f .* exp(-1j * 2 * pi * fAxis * tau);
    refDelayed_padded = ifft(Shifted_f);

    refDelayed = refDelayed_padded(1:N);

    t = (0:N-1)' / fs;
    baseEcho = refDelayed .* exp(1j * 2 * pi * fd * t);

    alpha = sum(surv(:) .* conj(baseEcho)) / sum(abs(baseEcho).^2);
    simEcho = alpha * baseEcho;

    surv_clean = surv;
    surv_clean(:) = surv(:) - simEcho;

    fprintf("CLEAN: range: %.2f km, vel: %.2f m/s\n", r_km, v_ms);
end
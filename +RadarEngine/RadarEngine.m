classdef RadarEngine < handle
    properties (Access = public)
        SignalParams
        filtParams
        CafMap
        CafMapLin
        corr
        ProcessingFlags
        CleanRmPow = 0
        RangeAxis
        DopplerAxis
        xRef
        xSurv
        xSurvRaw
        MinRange = -5
        MaxRange = 5
        MinDoppler = -100
        MaxDoppler = 100
        DecimationFactor = 1000
    end

    properties (Access = private)
        m_Filter1
        m_Filter2
    end

    events
        DataProcessed
    end

    methods (Access = public)
        function obj = RadarEngine(processingFlags)
            obj.ProcessingFlags = processingFlags;
        end

        function setSignals(obj, ref, surv)
            obj.xRef = ref;
            obj.xSurvRaw = surv;
        end

        function setCafParams(obj, minRange, maxRange, minDoppler, maxDoppler, decFactor)
            obj.MinRange = minRange;
            obj.MaxRange = maxRange;
            obj.MinDoppler = minDoppler;
            obj.MaxDoppler = maxDoppler;
            obj.DecimationFactor = decFactor;
        end

        function processSignals(obj, params)
            ref = obj.xRef;
            surv = obj.xSurvRaw;

            if obj.ProcessingFlags.useFilter1 && ~isempty(obj.m_Filter1)
                surv = obj.m_Filter1.apply(ref, surv);
                %surv(1:2e3) = eps + eps*1j;
                %surv(end-2e3:end) = eps + eps*1j;
            end

            if obj.ProcessingFlags.useFilter2 && ~isempty(obj.m_Filter2)
                surv = obj.m_Filter2.apply(ref, surv);
                surv(1:2e3) = eps + eps*1j;
                surv(end-2e3:end) = eps + eps*1j;
            end

            obj.calculateCAF(ref, surv, params(1).fs);

            if obj.ProcessingFlags.useClean
                [surv, r_km, v_ms] = obj.applyClean(ref, surv, params(1).fs, params(1).fc(1));
                r_km
                v_ms
                obj.calculateCAF(ref, surv, params(1).fs);
            end
            obj.corr = xcorr(surv, ref, 1000);
            obj.xSurv = surv;
            
            disp("Calculating CAF: DONE");
            notify(obj, 'DataProcessed');
        end

        function calculateCAF(obj, ref, surv, fs)
            c = 3e8;
            N = length(ref);

            numBlocks = 512;
            samplesPerBlock = floor(N / numBlocks);

            ref_2D = reshape(ref(1:samplesPerBlock*numBlocks), samplesPerBlock, numBlocks);
            surv_2D = reshape(surv(1:samplesPerBlock*numBlocks), samplesPerBlock, numBlocks);

            winRange = hann(samplesPerBlock);
            winDoppler = hann(numBlocks)';

            R_f = fft(ref_2D, [], 1);
            S_f = fft(surv_2D, [], 1);
            epsilon = 1e-8 * max(abs(R_f), [], 'all');
            %cross_spec = (S_f .* conj(R_f)) ./ (abs(R_f).^2 + epsilon);
            cross_spec = (S_f .* conj(R_f));
            cross_spec = cross_spec .* winRange;
            corr_fast = ifft(cross_spec, [], 1);

            corr_fast_win = corr_fast .* winDoppler;
            caf_matrix = fft(corr_fast_win, [], 2);

            caf_matrix = fftshift(caf_matrix, 2);
            caf_matrix = fftshift(caf_matrix, 1);

            %caf_dB = 10 * log10(abs(caf_matrix) + eps);
            caf_dB = mag2db(abs(caf_matrix) + eps);
            tau_full = linspace(-samplesPerBlock/2, samplesPerBlock/2 - 1, samplesPerBlock)' / fs;
            range_full = (tau_full * c) / 1000;

            T_block = samplesPerBlock / fs;
            F_prf = 1 / T_block;
            doppler_full = linspace(-F_prf/2, F_prf/2, numBlocks);

            rangeIdx = range_full >= obj.MinRange & range_full <= obj.MaxRange;
            dopplerIdx = doppler_full >= obj.MinDoppler & doppler_full <= obj.MaxDoppler;

            obj.CafMapLin = caf_matrix(rangeIdx, dopplerIdx);
            obj.CafMap = caf_dB(rangeIdx, dopplerIdx);
            obj.RangeAxis = range_full(rangeIdx);
            obj.DopplerAxis = doppler_full(dopplerIdx);
        end

        function [x_surv_clean, bistatic_range_km, bistatic_velocity] = applyClean(obj, x_ref, x_surv, fs, fc)
            c = 3e8;
            lambda = c / fc;
            N = length(x_ref);

            mag_matrix = abs(obj.CafMapLin);
            [maxPeak, linear_idx] = max(mag_matrix, [], 'all');
            [r_idx, d_idx] = ind2sub(size(mag_matrix), linear_idx);

            delta_doppler = obj.interpolate_3point(mag_matrix, r_idx, d_idx, 2);
            delta_delay   = obj.interpolate_3point(mag_matrix, r_idx, d_idx, 1);

            r_idx_float = r_idx + delta_delay;
            d_idx_float = d_idx + delta_doppler;

            samplesPerBlock = size(mag_matrix, 1);
            numBlocks = size(mag_matrix, 2);

            tau_samples = (r_idx_float - 1) - (samplesPerBlock / 2);
            bistatic_range_km = (tau_samples / fs) * c / 1000;

            T_block = samplesPerBlock / fs;
            F_prf = 1 / T_block;
            doppler_step = F_prf / numBlocks;
            fd = -F_prf/2 + (d_idx_float - 1) * doppler_step;

            bistatic_velocity = fd * (lambda / 2);

            d_int = floor(tau_samples);
            d_frac = tau_samples - d_int;

            L_kernel = 30;
            n_k = -L_kernel:L_kernel;
            h = sinc(n_k - d_frac) .* hann(2*L_kernel+1)';
            h = h / sum(h);

            if abs(d_int) >= N
                x_ref_final = zeros(N,1);
            else
                if d_int >= 0
                    temp = [zeros(d_int,1); x_ref(1:N-d_int)];
                else
                    temp = [x_ref(1-d_int:end); zeros(-d_int,1)];
                end
                x_ref_final = conv(temp, h, 'same');
                x_ref_final = x_ref_final(1:N);
            end

            t_vec = (0:N-1).' / fs;
            doppler_phasor = exp(1i * 2 * pi * fd * t_vec);
            echo_model = x_ref_final .* doppler_phasor;

            denom = echo_model' * echo_model;
            if abs(denom) < 1e-12
                alpha = 0;
            else
                alpha = (echo_model' * x_surv) / denom;
            end

            echo_estimated = alpha * echo_model;
            x_surv_clean = x_surv - echo_estimated;
            rawPow = 10 * log10(mean(abs(x_surv).^2) + eps);
            cleanPow = 10 * log10(mean(abs(x_surv_clean).^2) + eps);
            obj.CleanRmPow = rawPow - cleanPow;
            
        end
        function delta = interpolate_3point(obj, M, r, c, dim)
            [rows, cols] = size(M);
            if dim == 1
                if r <= 1 || r >= rows, delta=0; return; end
                vals = M(r-1 : r+1, c);
            else
                if c <= 1 || c >= cols, delta=0; return; end
                vals = M(r, c-1 : c+1).';
            end

            y1 = vals(1); y2 = vals(2); y3 = vals(3);
            denom = 2 * (y1 - 2*y2 + y3);

            if abs(denom) < 1e-10, delta = 0;
            else, delta = (y1 - y3) / denom; end

            if abs(delta) > 0.6, delta = 0; end
        end

        function setProcessingFlags(obj, flags)
            obj.ProcessingFlags = flags;
        end

        function setFilters(obj, filter1Name, params1Struct, filter2Name, params2Struct)
            switch filter1Name
                case "ECA"
                    obj.m_Filter1 = Algorithms.Filters.EcaFilter();
                case "Lattice"
                    obj.m_Filter1 = Algorithms.Filters.LatticeFilter();
                case "Fast ECA"
                    obj.m_Filter1 = Algorithms.Filters.FastEcaFilter();
            end
            obj.m_Filter1.setParams(params1Struct);
            obj.filtParams.params1 = params1Struct;
            switch filter2Name
                case "ECA"
                    obj.m_Filter2 = Algorithms.Filters.EcaFilter();
                case "Lattice"
                    obj.m_Filter2 = Algorithms.Filters.LatticeFilter();
                case "Fast ECA"
                    obj.m_Filter2 = Algorithms.Filters.FastEcaFilter();
            end
            obj.m_Filter2.setParams(params2Struct);
            obj.filtParams.params2 = params2Struct;
        end

    end
end
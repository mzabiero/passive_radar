classdef RadarEngine < handle
    properties (Access = public)
        SignalParams
        filtParams
        CafMap
        CafMapClean
        CafMapLin
        CleanIters = 1
        corr
        ProcessingFlags RadarEngine.ProcessingFlags
        CleanRmPow = 0
        RangeAxis
        DopplerAxis
        xRef
        xSurv
        xSurvCAF
        xSurvRaw
        MinRange = -5
        MaxRange = 5
        MinDoppler = -100
        MaxDoppler = 100
        DecimationFactor = 1000
        debugPlot_delay
        debugPlot_doppler
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
            end
            if obj.ProcessingFlags.useFilter2 && ~isempty(obj.m_Filter2)
                surv = obj.m_Filter2.apply(ref, surv);
            end

            minR = obj.MinRange;
            maxR = obj.MaxRange;
            minVel = obj.MinDoppler;
            maxVel = obj.MaxDoppler;
            fs = params(1).fs;
            fc = params(1).fc(1);

            %cafFunc = @obj.calculateCAF;
            cafFunc = @Algorithms.calcCafBatched;
            if obj.ProcessingFlags.CafType == "Direct"
                cafFunc = @Algorithms.calcCafDirect;
            end

            [cafLin, rAx, vAx] = cafFunc(ref, surv, fs, fc, minR, maxR, minVel, maxVel);
            cafDb = mag2db(abs(cafLin) + eps);

            obj.CafMapLin = cafLin;
            cafLinClean = cafLin;
            obj.CafMap = cafDb;
            obj.RangeAxis = rAx;
            obj.DopplerAxis = vAx;

            surv_clean = surv;
            obj.xSurvCAF = surv;
            
            if obj.ProcessingFlags.useClean
                currentCafDb = cafDb;
                for i = 1:obj.CleanIters
                    [~, linearIdx] = max(currentCafDb, [], 'all');
                    [rIdx, vIdx] = ind2sub(size(currentCafDb), linearIdx);
    

                    %[surv_clean, ~, ~] = obj.applyClean(ref, surv_clean, fs, fc(1), rIdx, vIdx);
                    [surv_clean,~,~] = CLEAN(cafLinClean, rAx, vAx, ref, surv_clean, fs, fc, [], [], true);
                    [cafLinClean, ~, ~] = cafFunc(ref, surv_clean, fs, fc, minR, maxR, minVel, maxVel);
                    currentCafDb = mag2db(abs(cafLinClean) + eps);
                end
                obj.CafMapClean = currentCafDb;
            else
                obj.CafMapClean = 0;
            end

            obj.corr = xcorr(surv_clean, ref, 1000);
            obj.xSurv = surv_clean;

            rawPow = 10 * log10(mean(abs(surv).^2) + eps);
            cleanPow = 10 * log10(mean(abs(surv_clean).^2) + eps);
            obj.CleanRmPow = rawPow - cleanPow;

            disp("Calculating CAF: DONE");
            notify(obj, 'DataProcessed');
        end

        function [caf, rax, vax] = calculateCAF(obj, ref, surv, fs,fc,minR, maxR, minVel, maxVel, cleanFlag)
            if nargin < 10
                cleanFlag = false;
            end

            c = 3e8;
            N = length(ref);

            max_R = max(abs(obj.MaxRange), abs(obj.MinRange));
            Q = ceil((max_R * 1000 / c) * fs) + 1;
            Q = max(Q, 64);

            P = floor(N / Q);
            if P < 1
                error('Sygnal zbyt krotki dla zadanego promienia MaxRange.');
            end

            samplesPerBlock = Q;
            numBlocks = P;

            ref_2D = reshape(ref(1:samplesPerBlock*numBlocks), samplesPerBlock, numBlocks);
            surv_2D = reshape(surv(1:samplesPerBlock*numBlocks), samplesPerBlock, numBlocks);

            N_fast = 2 * samplesPerBlock;
            winRange = hann(N_fast);
            winDoppler = hann(numBlocks)';

            R_f = fft(ref_2D, N_fast, 1);
            S_f = fft(surv_2D, N_fast, 1);

            cross_spec = (S_f .* conj(R_f));
            cross_spec = cross_spec .* winRange;
            corr_fast = ifft(cross_spec, [], 1);
            corr_fast_win = corr_fast .* winDoppler;

            caf_matrix = fft(corr_fast_win, [], 2);
            caf_matrix = fftshift(caf_matrix, 2);
            caf_matrix = fftshift(caf_matrix, 1);

            caf_dB = mag2db(abs(caf_matrix) + eps);

            tau_full = linspace(-N_fast/2, N_fast/2 - 1, N_fast)' / fs;
            range_full = (tau_full * c) / 1000;

            T_block = samplesPerBlock / fs;
            F_prf = 1 / T_block;
            doppler_full = linspace(-F_prf/2, F_prf/2, numBlocks);

            rangeIdx = range_full >= obj.MinRange & range_full <= obj.MaxRange;
            dopplerIdx = doppler_full >= obj.MinDoppler & doppler_full <= obj.MaxDoppler;

            obj.CafMapLin = caf_matrix(rangeIdx, dopplerIdx);

            if cleanFlag
                obj.CafMapClean = caf_dB(rangeIdx, dopplerIdx);
            else
                obj.CafMap = caf_dB(rangeIdx, dopplerIdx);
            end
            caf = caf_dB;
            rax = range_full;
            vax = doppler_full;
            obj.RangeAxis = range_full(rangeIdx);
            obj.DopplerAxis = doppler_full(dopplerIdx);
        end

        function [x_surv_clean, bistatic_range_km, bistatic_velocity] = applyClean(obj, x_ref, x_surv, fs, fc, target_r, target_c)
            c = 3e8;
            lambda = c / fc;
            N = length(x_ref);
            mag_matrix = abs(obj.CafMapLin);

            if nargin >= 6 && ~isempty(target_r) && ~isempty(target_c)
                search_win = 3;
                r_start = max(1, target_r - search_win);
                r_end   = min(size(mag_matrix, 1), target_r + search_win);
                d_start = max(1, target_c - search_win);
                d_end   = min(size(mag_matrix, 2), target_c + search_win);

                local_window = mag_matrix(r_start:r_end, d_start:d_end);
                [~, max_idx_local] = max(local_window, [], 'all');
                [r_local, d_local] = ind2sub(size(local_window), max_idx_local);
                r_idx = r_start + r_local - 1;
                d_idx = d_start + d_local - 1;
            else
                [~, linear_idx] = max(mag_matrix, [], 'all');
                [r_idx, d_idx] = ind2sub(size(mag_matrix), linear_idx);
            end

            if r_idx > 1 && r_idx < size(mag_matrix, 1)
                a_r = mag_matrix(r_idx-1, d_idx);
                b_r = mag_matrix(r_idx, d_idx);
                c_r = mag_matrix(r_idx+1, d_idx);
                delta_delay = 0.5 * (a_r - c_r) / (a_r - 2*b_r + c_r);
            else
                delta_delay = 0;
            end

            if d_idx > 1 && d_idx < size(mag_matrix, 2)
                a_d = mag_matrix(r_idx, d_idx-1);
                b_d = mag_matrix(r_idx, d_idx);
                c_d = mag_matrix(r_idx, d_idx+1);
                delta_doppler = 0.5 * (a_d - c_d) / (a_d - 2*b_d + c_d);
            else
                delta_doppler = 0;
            end

            r_idx_float = r_idx + delta_delay;
            d_idx_float = d_idx + delta_doppler;

            range_step_km = obj.RangeAxis(2) - obj.RangeAxis(1);
            doppler_step_hz = obj.DopplerAxis(2) - obj.DopplerAxis(1);

            bistatic_range_km = obj.RangeAxis(1) + (r_idx_float - 1) * range_step_km;
            fd = obj.DopplerAxis(1) + (d_idx_float - 1) * doppler_step_hz;

            tau_samples = (bistatic_range_km * 1000 / c) * fs;
            bistatic_velocity = -fd * lambda; 

            d_int = floor(tau_samples);
            d_frac = tau_samples - d_int;

            L_kernel = 30;
            n_k = (-L_kernel:L_kernel)';
            h = sinc(n_k - d_frac) .* hann(2*L_kernel+1);

            x_ref_frac = conv(x_ref, h, 'same');

            if abs(d_int) >= N
                x_ref_final = zeros(N, 1);
            else
                if d_int >= 0
                    x_ref_final = [zeros(d_int, 1); x_ref_frac(1:N-d_int)];
                else
                    x_ref_final = [x_ref_frac(1-d_int:end); zeros(-d_int, 1)];
                end
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
        function [tfMap, timeAxis, dopplerAxis, slow_time_sig] = calculateRadarSTFT(obj, ref, surv, fs, rangeOffsetKm, params, useWVD, rangeMargin)
            if nargin < 8
                rangeMargin = 0;
            end
            if nargin < 7
                useWVD = false;
            end
            if nargin < 6 || isempty(params)
                params = struct();
                params.samplesPerBlock = 4096;
                params.windowLength = 58;
                params.overlapLength = 52;
                params.nfft = 1024;
            end

            c = 3e8;
            Q = params.samplesPerBlock;
            P = floor(min(length(ref), length(surv)) / Q);

            ref_2D = reshape(ref(1:Q*P), Q, P);
            surv_2D = reshape(surv(1:Q*P), Q, P);

            N_fast = 2 * Q;
            winRange = hann(N_fast);

            R_f = fft(ref_2D, N_fast, 1);
            S_f = fft(surv_2D, N_fast, 1);

            cross_spec = (S_f .* conj(R_f)) .* winRange;
            corr_fast = ifft(cross_spec, [], 1);
            corr_fast_shifted = fftshift(corr_fast, 1);

            tau_full = linspace(-N_fast/2, N_fast/2 - 1, N_fast)' / fs;
            range_full = (tau_full * c) / 1000;

            [~, rIdx] = min(abs(range_full - rangeOffsetKm));

            rStart = max(1, rIdx - rangeMargin);
            rStop = min(N_fast, rIdx + rangeMargin);

            if rangeMargin > 0
                matrixSlice = corr_fast_shifted(rStart:rStop, :);
                slow_time_sig = reshape(matrixSlice.', 1, []);
            else
                slow_time_sig = corr_fast_shifted(rIdx, :);
            end

            F_prf = fs / Q;

            if useWVD
                [S, dopplerAxis, timeAxis] = wvd(slow_time_sig, F_prf, 'smoothedPseudo');
                tfMap = 10 * log10(abs(S) + eps);
            else
                [S, dopplerAxis, timeAxis] = spectrogram(slow_time_sig, params.windowLength, params.overlapLength, params.nfft, F_prf, 'centered');
                tfMap = mag2db(abs(S) + eps);
            end
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
classdef RadarEngine < handle
    properties (Access = public)
        SignalParams
        filtParams
        CafMap
        corr
        ProcessingFlags
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
        m_Filter
        m_FilterShort
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

            if obj.ProcessingFlags.useFilter && ~isempty(obj.m_Filter)
                surv = obj.m_Filter.apply(ref, surv);
                surv(1:2e3) = eps + eps*1j;
                surv(end-2e3:end) = eps + eps*1j;
            end

            obj.calculateCAF(ref, surv, params(1).fs);

            obj.corr = xcorr(surv, ref, 1000);
            disp("Calculating CAF: DONE");
            notify(obj, 'DataProcessed');
            obj.xSurv = surv;
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
            epsilon = 1e-6 * max(abs(R_f), [], 'all'); 
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

            obj.CafMap = caf_dB(rangeIdx, dopplerIdx);
            obj.RangeAxis = range_full(rangeIdx);
            obj.DopplerAxis = doppler_full(dopplerIdx);
        end

        function setProcessingFlags(obj, flags)
            obj.ProcessingFlags = flags;
        end

        function setFilter(obj, filterName, paramsStruct)
            switch filterName
                case "ECA"
                    obj.m_Filter = Algorithms.Filters.EcaFilter();
                case "Lattice"
                    obj.m_Filter = Algorithms.Filters.LatticeFilter();
                case "Fast ECA"
                    obj.m_Filter = Algorithms.Filters.FastEcaFilter();
            end
            obj.m_Filter.setParams(paramsStruct);
            obj.filtParams = paramsStruct;
        end

    end
end
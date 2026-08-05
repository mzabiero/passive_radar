classdef RadarEngine < handle
    properties (Access = public)
        FilterParams
        SignalParams
        CafMap
        corr
        ProcessingFlags RadarEngine.ProcessingFlags
        RangeAxis
        DopplerAxis
        xRef
        xSurv
    end
    
    properties (Access = private)
        m_EcaFilter Algorithms.EcaFilter
    end 

    events
        DataProcessed
    end

    methods (Access = public)
        function obj = RadarEngine(processingFlags, filterParams)
            obj.ProcessingFlags = RadarEngine.ProcessingFlags(processingFlags);
            obj.m_EcaFilter = Algorithms.EcaFilter(filterParams);
        end
        function setSignals(obj, ref, surv)
            obj.xRef = ref;
            obj.xSurv = surv;
        end
        function processSignals(obj, params)
            
            ref = obj.xRef;
            surv = obj.xSurv;
            if obj.ProcessingFlags.useEca
                surv = obj.m_EcaFilter.apply(ref, surv);
            end

            Fs = params.fs; 
            c = 3e8; 
            N = length(ref);

            numBlocks = 516; 
            samplesPerBlock = floor(N / numBlocks); 

            ref_2D = reshape(ref(1:samplesPerBlock*numBlocks), samplesPerBlock, numBlocks);
            surv_2D = reshape(surv(1:samplesPerBlock*numBlocks), samplesPerBlock, numBlocks);

            R_f = fft(ref_2D, [], 1);
            S_f = fft(surv_2D, [], 1);
            corr_fast = ifft(S_f .* conj(R_f), [], 1);

            caf_matrix = fft(corr_fast, [], 2);
            caf_matrix = fftshift(caf_matrix, 2); 

            caf_dB = 10 * log10(abs(caf_matrix) + eps);

            tau = (0 : samplesPerBlock-1)' / Fs; 
            range_km = (tau * c) / 1000;

            T_block = samplesPerBlock / Fs; 
            F_prf = 1 / T_block; 
            doppler_hz = linspace(-F_prf/2, F_prf/2, numBlocks);

            maxRangeKm = 150;
            [~, maxBin] = min(abs(range_km - maxRangeKm)); 
            obj.CafMap = caf_dB(1:maxBin, :);
            obj.RangeAxis = range_km(1:maxBin);
            obj.DopplerAxis = doppler_hz;
            
            xcor =  xcorr(ref,surv, 1000);
            lags = -1000 : 1000;
            notify(obj, 'DataProcessed');
            obj.xSurv = surv;
        end

        function setProcessingFlags(obj, flags)
            obj.ProcessingFlags = flags;
        end

        function setFilterParams(obj, filtLen)
            obj.m_EcaFilter.FilterLength = filtLen;
        end
    end
end
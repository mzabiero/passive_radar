classdef RadarEngine < handle
    properties (Access = public)
        FilterParams
        SignalParams
        CafMap
        ProcessingFlags RadarEngine.ProcessingFlags
    end

    events
        DataProcessed
    end

    methods (Access = public)
        function obj = RadarEngine()
            
        end

        function processSignals(ref, surv)
            cafMap = abs(fftshift(fft2(ref .* conj(surv))));
            obj.CafMap = cafMap;
            notify(obj, 'DataProcessed');
        end
        
        function setProcessingFlags(obj, flags)
            obj.ProcessingFlags = flags;
        end
    end
end
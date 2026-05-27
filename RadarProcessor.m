classdef RadarProcessor < handle

    properties (Access = public)
        Fs (1,1) double = 10e6;     % 10MHz for DVB-T channel
        Fc (1,1) double = 500e6;    % popular DVB-t station
        x_ref (:,1) double
        x_surv (:,1) double

        CAF_matrix double
    end

    methods (Access = public)
        function obj = RadarProcessor(initialFs, initialFc)
            obj.Fs = initialFs;
            obj.Fc = initialFc;
        end
        
        function obj = loadRef(ref)
            obj.x_ref = ref;
        end
        
        function obj = loadSurv(surv)
            obj.x_surv = surv;
        end

        function obj = loadSignals(ref,surv)
            obj.x_ref = ref;
            obj.x_surv = surv;
        end

        function obj = calculateCAF()
            
        end
    end
end
classdef RadarEngine < handle
    properties (Access = public)
        FilterParams
        SignalParams
        CafMap
    end

    events
        DataProcessed
    end

    methods (Access = public)
        function obj = RadarEngine()
            
        end

    end
end
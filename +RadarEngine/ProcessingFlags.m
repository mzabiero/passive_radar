classdef ProcessingFlags < handle
    properties 
        useFilter1
        useFilter2
        isWindow  
        trimSignals 
        useClean
        CafType
        CLEANint3p
    end

    methods
        function obj = ProcessingFlags(flags)
            if nargin < 1
                obj.useFilter1 = false;
                obj.useFilter2 = false;
                obj.isWindow = false;
                obj.trimSignals = false;
                obj.useClean = false;
                obj.CafType = "Batched";
                obj.CLEANint3p = "Interpolate";
            else
                obj.useFilter1 = flags.useFilter1;
                obj.useFilter2 = flags.useFilter1;
                obj.isWindow = flags.isWindow;
                obj.trimSignals = flags.trimSignals;
                obj.useClean = flags.useClean;
                obj.CafType = flags.CafType;
                obj.CLEANint3p = flags.CLEANint3p;
            end
        end
    end
end
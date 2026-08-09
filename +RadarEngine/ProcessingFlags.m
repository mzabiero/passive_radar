classdef ProcessingFlags < handle
    properties 
        useFilter
        isWindow  
        trimSignals 
    end

    methods
        function obj = ProcessingFlags(flags)
            if nargin < 1
                obj.useFilter = false;
                obj.isWindow = false;
                obj.trimSignals = false;
            else
                obj.useFilter = flags.useFilter;
                obj.isWindow = flags.isWindow;
                obj.trimSignals = flags.trimSignals;
            end
        end
    end
end
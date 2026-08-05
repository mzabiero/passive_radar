classdef ProcessingFlags < handle
    properties 
        isWindow 
        useEca 
        trimSignals 
    end

    methods
        function obj = ProcessingFlags(flags)
            if nargin < 1
                obj.isWindow = false;
                obj.useEca = false;
                obj.trimSignals = false;
            else
                obj.isWindow = flags.isWindow;
                obj.useEca = flags.useEca;
                obj.trimSignals = flags.trimSignals;
            end
        end
    end
end
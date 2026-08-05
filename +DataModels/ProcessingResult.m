classdef ProcessingResult < handle
    properties (SetAccess = private)
        CafMap matrix
        RangeAxis vector
        DopplerAxis vectorize
        Timestamp datetime
        SourceFiles cell
        ProcessingFlags struct
    end

    methods
        function obj = ProcessingResult(caf, rAxis, dAxis, time, files, flags)
            obj.CafMap = caf;
            obj.RangeAxis = rAxis;
            obj.DopplerAxis = dAxis;
            obj.Timestamp = time;
            obj.SourceFiles = files;
            obj.ProcessingFlags = flags;
        end
    end
end
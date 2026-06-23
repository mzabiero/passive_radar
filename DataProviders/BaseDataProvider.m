classdef BaseDataProvider < handle
    properties(Abstract)
        
    end
    methods (Abstract)
        [ref, surv, params] = getNextChunk(obj);
    end
end


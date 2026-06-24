classdef BaseDataProvider < handle
    properties(Abstract)
        
    end
    methods (Abstract)
        [ref, surv, params, success] = getNextChunk(obj);
        
    end
end


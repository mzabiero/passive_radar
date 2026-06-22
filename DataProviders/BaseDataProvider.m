classdef BaseDataProvider < handle
    methods (Abstract)
        [ref, surv, params] = getNextChunk(obj);
    end
end


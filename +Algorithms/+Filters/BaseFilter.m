classdef BaseFilter < handle
    properties
        filterLength
        backFiltLength (1,1) uint32 = 0
    end

    methods (Abstract)
        survClean = apply(obj, ref, surv)
        setParams(obj,params);
    end
end
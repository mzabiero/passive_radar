classdef (Abstract) BaseFileParser < handle
    properties
        VarMapping struct
    end

    methods (Abstract)
        [ref, surv, metadata, success] = parseFile(obj, filePath)
    end
end
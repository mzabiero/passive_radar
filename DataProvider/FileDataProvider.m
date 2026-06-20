classdef FileDataProvider < BaseDataProvider
    properties
        FilePaths
        CurrentIndex = 1;
    end

    methods
        function obj = FileDataProvider(path)
            obj.FilePaths = path;
        end
        function [ref,surv,params] = getNextChunk(obj)
            if obj.CurrentIndex <= length(obj.FilePaths)
                data = load(obj.FilePaths{obj.CurrentIndex});
                ref = data.ref;
                surv = data.surv;
                params = data.params;
                obj.CurrentIndex = obj.CurrentIndex + 1;
            else
                ref = [];
                surv = [];
                params = {};
            end
        end
    end
end
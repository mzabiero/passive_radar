classdef FileDataProvider < BaseDataProvider
    properties
        FilesPath
        CurrentIndex = 1;
        NumFiles
    end

    methods
        function obj = FileDataProvider(path)
            if nargin >= 1
                obj.FilesPath = path;
            else
                return;
            end
        end

        function [ref,surv,params] = getNextChunk(obj)
            if obj.CurrentIndex <= length(obj.FilesPath)
                data = load(obj.FilesPath{obj.CurrentIndex});
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

        function numFiles =  getNumFiles(obj)
            if isempty(obj.FilesPath) 
                numFiles = NaN;
            else
                numFiles = length(obj.FilesPath);
            end
        end
    end
end
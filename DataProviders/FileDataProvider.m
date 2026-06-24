classdef FileDataProvider < BaseDataProvider
    properties (SetAccess = private, GetAccess = public)
        numFiles (1,1) double = 0
        CurrentIndex (1,1) double = 1
    end
    properties (Access = private)
        m_filesPath cell = {}
        m_activeParser
    end

    methods (Access = public)
        function obj = FileDataProvider(path)
            if nargin >= 1
                obj.FilesPath = path;
            else
                return;
            end
        end
        function configure(obj,filesPath,mappingConfig)
            if isempty(filesPath) return; end
            obj.m_filesPath = filesPath;
            obj.numFiles    = length(filesPath);
            obj.CurrentIndex = 1;
            if isfield(mappingConfig, 'DataSource')
                switch mappingConfig.DataSource
                    case 'Is3p'
                        obj.m_activeParser = Parsers.Is3pFileParser();
                    otherwise
                        error("Unknown files format: %s", mappingConfig.DataSource);
                end
            end
        end

        function [ref,surv,params, success] = getNextChunk(obj)
            if obj.CurrentIndex <= obj.numFiles
                [ref, surv, params, success] = obj.m_activeParser.parseFile(obj.m_filesPath(obj.CurrentIndex));
                obj.CurrentIndex = obj.CurrentIndex + 1;
            else
                ref = [];
                surv = [];
                params = {};
            end
        end

        function numFiles =  getnumFiles(obj)
            if isempty(obj.FilesPath) 
                numFiles = NaN;
            else
                numFiles = length(obj.FilesPath);
            end
        end
    end
end
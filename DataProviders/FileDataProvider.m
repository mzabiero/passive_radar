classdef FileDataProvider < BaseDataProvider
    properties (Access = public)
        CurrentIndex (1,1) double = 1
    end
    properties (SetAccess = private, GetAccess = public)
        numFiles (1,1) double = 0
    end
    properties (Access = private)
        m_filesPath cell = cell(2,2)
        m_activeParser
    end

    methods (Access = public)
        function obj = FileDataProvider(path)
            if nargin >= 1
                obj.m_filesPath = path;
            else
                return;
            end
        end
        function configure(obj,filesPath,mappingConfig)
            if isempty(filesPath), return; end
            obj.numFiles    = length(filesPath);
            obj.CurrentIndex = 1;
            if isfield(mappingConfig, 'DataSource')
                switch mappingConfig.DataSource
                    case 'Is3p'
                        obj.m_filesPath = filesPath;
                        obj.m_activeParser = Parsers.Is3pFileParser();
                    case 'Ref'
                        obj.m_filesPath{1} = filesPath;
                        obj.m_activeParser = Parsers.BinFileParser();
                        obj.m_activeParser.IsSimulation = mappingConfig.isSimulation;
                    case 'Surv'
                        obj.m_filesPath{2} = filesPath;
                        obj.m_activeParser = Parsers.BinFileParser();
                        obj.m_activeParser.IsSimulation = mappingConfig.isSimulation;
                    otherwise
                        error("Unknown files format: %s", mappingConfig.DataSource);
                end
            end
        end

        function [ref,surv,params, success, fileParams] = getNextChunk(obj)
            if obj.CurrentIndex <= obj.numFiles
                if isa(obj.m_activeParser, 'Parsers.BinFileParser')
                    currRef = obj.m_filesPath{1}(obj.CurrentIndex);
                    currSurv = obj.m_filesPath{2}(obj.CurrentIndex);
                    targetPath = {currRef, currSurv};
                    fileNameLog = strcat(string(currRef), "|", string(currSurv)); 
                else
                    targetPath = obj.m_filesPath{obj.CurrentIndex};
                    fileNameLog = string(targetPath);
                end
                [ref, surv, params, success] = obj.m_activeParser.parseFile(targetPath);
                fileParams = struct("filename", fileNameLog, "fileIdx", obj.CurrentIndex);
                obj.CurrentIndex = obj.CurrentIndex + 1;
            else
                success = 0;
                fileParams = 0;
                ref = [];
                surv = [];
                params = {};
                disp('End of files');
            end
        end
        function fnames = getCurrentFilenames(obj)
            if isa(obj.m_activeParser,'Parsers.BinFileParser')
                fnames = struct;
                if obj.CurrentIndex >= obj.numFiles
                    idx = obj.numFiles;
                else
                    idx = obj.CurrentIndex;
                end
                fnames.ref = obj.m_filesPath{1,idx};
                fnames.surv = obj.m_filesPath{2,idx};
            else
                fnames = obj.m_filesPath{obj.CurrentIndex};
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

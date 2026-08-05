classdef HistoryManager < handle
    properties (Access = private)
        SaveDirectory string
        Buffer cell
        BatchSize (1,1) double = 50
    end
    methods
        function obj = HistoryManager(directory)
            obj.SaveDirectory = string(directory);
            if ~exist(obj.SaveDirectory, 'dir'), mkdir(obj.SaveDirectory); end
        end

        function addResult(obj, resultObj)
            obj.Buffer{end+1} = resultObj;
            if length(obj.Buffer) >= obj.BatchSize
                obj.flushToDisk();
            end
        end

        function flushToDisk(obj)
            if isempty(obj.Buffer), return; end
            
            fileName = fullfile(obj.SaveDirectory, sprintf('Batch_%s.mat', datetime('now', 'Format', 'HHmmss_SSS')));
            batchData = obj.Buffer;
            
            % Zapis z opcją -v7.3 dla dużych plików
            save(fileName, 'batchData', '-v7.3'); 
            obj.Buffer = {};
        end
    end
end
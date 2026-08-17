classdef BinFileParser < Parsers.BaseFileParser
    methods
        function [ref, surv, metadata, success] = parseFile(obj, filePath)
            ref = [];
            surv = [];
            metadata = struct('FileCount', 0, 'TotalSamples', 0, 'fs', 8e6, 'fc', 520e6);
            success = false;
            
            try
                if ~iscell(filePath) || length(filePath) ~= 2
                    error('BinFileParser:InvalidInput', 'Invalid filePath format.');
                end
                
                refPaths = filePath{1};
                survPaths = filePath{2};
                
                if ischar(refPaths) || isstring(refPaths)
                    refPaths = cellstr(refPaths);
                end
                if ischar(survPaths) || isstring(survPaths)
                    survPaths = cellstr(survPaths);
                end
                
                if length(refPaths) ~= length(survPaths)
                    error('BinFileParser:Mismatch', 'File count mismatch.');
                end
                
                ref = obj.loadAndConcatenate(refPaths);
                surv = obj.loadAndConcatenate(survPaths);
                
                minLen = min(length(ref), length(surv));
                ref = ref(1:minLen);
                surv = surv(1:minLen);
                
                success = true;
                metadata.FileCount = length(refPaths);
                metadata.TotalSamples = length(ref);
                
            catch ME
                success = false;
            end
        end
    end
    
    methods (Access = private)
        function outputSignal = loadAndConcatenate(obj, filePaths)
            totalSamples = 0;
            for i = 1:length(filePaths)
                fileInfo = dir(filePaths{i});
                totalSamples = totalSamples + floor(fileInfo.bytes / 8);
            end
            
            outputSignal = zeros(totalSamples, 1, 'like', 1i);
            currentIndex = 1;
            
            for i = 1:length(filePaths)
                fid = fopen(filePaths{i}, 'rb');
                if fid == -1
                    error('BinFileParser:FileOpenError', 'Cannot open file.');
                end
                
                raw = fread(fid, '*float32');
                fclose(fid);
                
                if mod(length(raw), 2) ~= 0
                    raw = raw(1:end-1);
                end
                
                complexSignal = complex(raw(1:2:end), raw(2:2:end));
                numSamples = length(complexSignal);
                
                outputSignal(currentIndex : currentIndex + numSamples - 1) = complexSignal;
                currentIndex = currentIndex + numSamples;
            end
        end
    end
end
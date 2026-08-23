classdef BinFileParser < Parsers.BaseFileParser
    properties
        IsSimulation (1,1) logical = false
    end
    
    methods
        function [ref, surv, metadata, success] = parseFile(obj, filePath)
            ref = [];
            surv = [];
            metadata = struct();
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
                
                [ref, refMeta] = obj.loadAndConcatenate(refPaths);
                [surv, ~] = obj.loadAndConcatenate(survPaths);
                
                minLen = min(length(ref), length(surv));
                ref = ref(1:minLen);
                surv = surv(1:minLen);
                
                metadata = refMeta;
                metadata.FileCount = length(refPaths);
                metadata.TotalSamples = minLen;
                
                if ~obj.IsSimulation
                    metadata.fs = 0;
                    metadata.fc = 0;
                end
                
                success = true;
            catch
                success = false;
            end
        end
    end
    
    methods (Access = private)
        function [outputSignal, firstMeta] = loadAndConcatenate(obj, filePaths)
            sigList = cell(length(filePaths), 1);
            firstMeta = struct();
            
            for i = 1:length(filePaths)
                [iq, meta] = obj.readSingleFile(filePaths{i});
                sigList{i} = iq(:);
                if i == 1
                    firstMeta = meta;
                end
            end
            outputSignal = vertcat(sigList{:});
        end
        
        function [iqSig, meta] = readSingleFile(obj, path)
            meta = struct();
            [~, ~, ext] = fileparts(path);
            
            if upper(ext) == ".MAT"
                data = load(path);
                if obj.IsSimulation
                    iqSig = data.iqSig;
                    meta.tx = data.tx;
                    meta.rx = data.rx;
                else
                    iqSig = struct2array(data);
                end
            else
                fid = fopen(path, 'rb');
                if fid == -1
                    error('BinFileParser:FileOpenError', 'Cannot open file.');
                end
                
                if obj.IsSimulation
                    metaLen = fread(fid, 1, 'uint32');
                    metaBytes = fread(fid, metaLen, '*uint8');
                    meta = jsondecode(native2unicode(metaBytes', 'UTF-8'));
                end
                
                raw = fread(fid, inf, 'float32');
                fclose(fid);
                
                if mod(length(raw), 2) ~= 0
                    raw = raw(1:end-1);
                end
                iqSig = raw(1:2:end) + 1j * raw(2:2:end);
            end
        end
    end
end
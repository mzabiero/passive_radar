classdef BinFileParser < Parsers.BaseFileParser
    properties
        IsSimulation (1,1) logical = false
        simSource string = "home"
    end
    
    methods
        function [ref, surv, metadata, success] = parseFile(obj, filePath)
            ref = [];
            surv = [];
            metadata = struct();
            success = false;
            %obj.simSource = simSource;
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
                
                if obj.IsSimulation && strcmp(obj.simSource, "work")
                    metaLen = fread(fid, 1, 'uint32');
                    metaBytes = fread(fid, metaLen, '*uint8');
                    meta = jsondecode(native2unicode(metaBytes', 'UTF-8'));
                    raw = fread(fid, inf, 'float32');
                    if mod(length(raw), 2) ~= 0
                        raw = raw(1:end-1);
                    end
                    iqSig = raw(1:2:end) + 1j * raw(2:2:end);
                elseif obj.IsSimulation && strcmp(obj.simSource, "home")
                    headerLen = fread(fid, 1, 'uint32');
                    headerChar = fread(fid, headerLen, '*char')';
                    meta = jsondecode(headerChar);

                    sigLen = fread(fid, 1, 'uint32');
                    realPart = fread(fid, sigLen, 'double');
                    imagPart = fread(fid, sigLen, 'double');
                    iqSig = realPart + 1i * imagPart;
                else
                    raw = fread(fid, 'float32');
                    iqSig = raw(1:2:end) + 1i * raw(2:2:end);
                end
                fclose(fid);
            end
        end
    end
end
classdef SimParser < Parsers.BaseFileParser
    methods (Static)
        function [ref, surv, metadata, success] = parseFile(obj, filePath)
            metadata = struct;
            [~, ~, ext] = fileparts(filePath);
            
            if upper(ext) == ".MAT"
                data = load(filePath);
                ref = data.iqSig;
                
                metadata.tx = data.tx;
                metadata.rx = data.rx;

            else
                fid = fopen(filePath, 'rb');
                
                metaLen = fread(fid, 1, 'uint32');
                metaBytes = fread(fid, metaLen, '*uint8');
                
                metadata = jsondecode(native2unicode(metaBytes', 'UTF-8'));

                rawData = fread(fid, inf, 'float32');
                fclose(fid);
                
                iqSig = rawData(1:2:end) + 1j * rawData(2:2:end);
            end
            success = true;
        end
    end
end

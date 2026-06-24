classdef Is3pFileParser < Parsers.BaseFileParser
    methods
        function [ref, surv, metadata, success] = parseFile(obj, filePath)
            try
                filePath = string(filePath);
                [data, header] = Utils.parseRaw(filePath);
                ref = 
                success = true;
            catch ME
                ref = []; surv = []; metadata = []; success = false;

                disp(ME.message);
            end
        end
    end

end
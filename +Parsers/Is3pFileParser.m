classdef Is3pFileParser < BaseFileParser
    methods
        function [ref, surv, metadata, success] = parseFile(obj, filePath)
            try
                [data, header] = Utils.parseRaw(filePath);
                
                success = true;
            catch ME
                ref = []; surv = []; metadata = []; success = false;

                disp(ME.message);
            end
        end
    end

end
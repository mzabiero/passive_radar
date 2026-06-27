classdef Is3pFileParser < Parsers.BaseFileParser
    methods
        function [ref, surv, metadata, success] = parseFile(~, filePath)
            try
                filePath = string(filePath);
                [data, header] = Utils.parseRaw(filePath);
                ref = data(:,2,:);
                surv = data(:,1,:);
                ref = ref(:);
                surv = surv(:);
                metadata = header;
                success = true;
            catch ME
                ref = []; surv = []; metadata = []; success = false;

                disp(ME.message);
            end
        end
    end

end
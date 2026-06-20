classdef CppBufferDataProvider < BaseDataProvider
    properties
        MemMap
    end
    methods
        function obj = CppBufferDataProvider()
            obj.MemMap = memmapfile('/dev/shm/pcl_buffer', 'format', 'single');
        end
        function [ref, surv, params] = getNextChunk(obj)
            rawData = obj.MemMap.Data;
            ref = rawData(1:100);  
            surv = rawData(101:200); 
            params = struct('chunkSize', 100); 
        end
    end
end
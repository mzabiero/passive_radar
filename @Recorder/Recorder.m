classdef Recorder < handle
    properties (Access = private)
        m_MatFile
        m_FrameCount (1,1) double = 0
        m_IsRecording (1,1) logical = false
    end
    
    methods (Access = public)
        function startRecording(obj, filePath, sourceFilename, filterName, filterParams, rangeAxis, dopplerAxis)
            obj.m_MatFile = matfile(filePath, 'Writable', true);
            
            metaData = struct();
            metaData.SourceFile = sourceFilename;
            metaData.RecordDate = datetime('now');
            
            obj.m_MatFile.MetaData = metaData;
            obj.m_MatFile.FilterName = filterName;
            obj.m_MatFile.FilterParams = filterParams;
            obj.m_MatFile.RangeAxis = rangeAxis;
            obj.m_MatFile.DopplerAxis = dopplerAxis;
            
            obj.m_FrameCount = 0;
            obj.m_IsRecording = true;
        end
        
        function addFrame(obj, cafMatrix)
            if ~obj.m_IsRecording
                return;
            end
            
            obj.m_FrameCount = obj.m_FrameCount + 1;
            obj.m_MatFile.CafMovie(:, :, obj.m_FrameCount) = single(cafMatrix);
        end
        
        function stopRecording(obj)
            obj.m_IsRecording = false;
            obj.m_MatFile = [];
        end
        
        function status = isRecording(obj)
            status = obj.m_IsRecording;
        end
    end
end
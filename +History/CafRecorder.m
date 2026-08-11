classdef CafRecorder < handle
    properties (Access = private)
        m_MatFile
        m_FrameCount (1,1) double = 0
        m_IsRecording (1,1) logical = false
        m_FramesFolder string
        m_DynamicRange (1,1) double = 50
    end
    
    methods (Access = public)
        function startRecording(obj,filePath, filterName)
            obj.m_MatFile = matfile(filePath, 'Writable', true);
            
            obj.m_MatFile.FilterName = filterName;
            
            [dirPath, name, ~] = fileparts(filePath);
            obj.m_FramesFolder = fullfile(dirPath, sprintf('%s_frames', name));
            if ~exist(obj.m_FramesFolder, 'dir')
                mkdir(obj.m_FramesFolder);
            end
            
            obj.m_FrameCount = 0;
            obj.m_IsRecording = true;
        end
        
        function addFrame(obj, cafMatrix, filterParams, rangeAxis, dopplerAxis, sourceFilename)
            if ~obj.m_IsRecording
                return;
            end
            
            obj.m_FrameCount = obj.m_FrameCount + 1;
            fc = obj.m_FrameCount;
            
            obj.m_MatFile.CafMovie(:, :, fc) = single(cafMatrix);
            
            obj.m_MatFile.FilterParams(fc, 1) = {filterParams};
            obj.m_MatFile.RangeAxes(fc, 1) = {rangeAxis};
            obj.m_MatFile.DopplerAxes(fc, 1) = {dopplerAxis};
            obj.m_MatFile.SourceFiles(fc, 1) = {sourceFilename};
            
            flippedCaf = flipud(cafMatrix);
            maxPeak = max(flippedCaf, [], 'all');
            minVal = maxPeak - obj.m_DynamicRange;
            
            cafNorm = (flippedCaf - minVal) / obj.m_DynamicRange;
            cafNorm = max(0, min(1, cafNorm));
            
            cmap = parula(256);
            imgRGB = ind2rgb(gray2ind(cafNorm, 256), cmap);
            
            fileName = sprintf('frame_%05d.png', fc);
            fullImgPath = fullfile(obj.m_FramesFolder, fileName);
            imwrite(imgRGB, fullImgPath);
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
classdef Recorder < handle
    properties (Access = private)
        m_MatFile
        m_FrameCount (1,1) double = 0
        m_IsRecording (1,1) logical = false
        m_FramesFolder string
        m_DynamicRange (1,1) double = 50
    end

    methods (Access = public)
        function startRecording(obj,filePath, filterName)
            if exist(filePath, 'file') == 2
                delete(filePath);
            end

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

            obj.m_MatFile.CafMovie(fc, 1) = {single(cafMatrix)};
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
    methods (Static)
        function playCafMovie(filePath)
            m = matfile(filePath);

            [numFrames, ~, ~] = size(m, 'CafMovie');

            fig = figure('Name', 'Odtwarzacz CAF', 'NumberTitle', 'off');
            ax = axes('Parent', fig);

            for i = 1:numFrames
                if ~ishghandle(ax)
                    break;
                end

                %cafFrame = m.CafMovie(:, :, i);
                tempCaf = m.CafMovie(i, 1);
                cafFrame = tempCaf{1};

                tempRange = m.RangeAxes(i, 1);
                tempDoppler = m.DopplerAxes(i, 1);
                tempSource = m.SourceFiles(i, 1);

                rangeAxis = tempRange{1};
                dopplerAxis = tempDoppler{1};
                sourceFile = tempSource{1};

                imagesc(ax, dopplerAxis, rangeAxis, cafFrame);

                ax.YDir = 'normal';
                axis(ax,'xy');
                colormap(ax,'jet'); colorbar(ax);
                maxPeak = max(cafFrame, [], 'all');
                meanPeax = mean(cafFrame,"all");
                %dynamicRange = 50;
                if isnan(maxPeak) || isempty(maxPeak) || isnan(meanPeax) || isempty(meanPeax)
                    return;
                end
                clim(ax, [meanPeax, maxPeak]);
                xlabel(ax, 'Doppler [Hz]');
                ylabel(ax, 'Range [km]');
                title(ax, sprintf('Klatka: %d / %d \n Plik nr: %d \n Nazwa pliku: %s\n', i, numFrames, sourceFile.fileIdx, sourceFile.filename), 'Interpreter', 'none');

                drawnow;

                %pause(0.1);
            end
        end

        function createVideoFromPngFolder(framesFolder, outputFilename, fps)
            filePattern = fullfile(framesFolder, '*.png');
            imageFiles = dir(filePattern);

            [~, idx] = sort([imageFiles.datenum]);
            imageFiles = imageFiles(idx);

            numFrames = length(imageFiles);
            if numFrames == 0
                error('Nie znaleziono plików PNG we wskazanym folderze.');
            end

            v = VideoWriter(outputFilename, "Motion JPEG AVI");
            v.FrameRate = fps;
            v.Quality = 95; 

            open(v);

            for i = 1:numFrames
                imgPath = fullfile(framesFolder, imageFiles(i).name);
                img = imread(imgPath);
                writeVideo(v, img);

                if mod(i, 100) == 0
                    fprintf('Przetworzono %d / %d klatek...\n', i, numFrames);
                end
            end

            close(v);
            fprintf('Zapis wideo zakończony: %s\n', outputFilename);
        end
    end
end
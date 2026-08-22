classdef Recorder < handle
    properties (Access = private)
        %m_MatFile
        m_FrameCount (1,1) double = 0
        m_IsRecording (1,1) logical = false
        m_FramesFolder string
        m_DynamicRange (1,1) double = 50
        
        % Uchwyty do renderowania w tle (niewidoczna figura)
        m_HiddenFig
        m_HiddenAx
        m_HiddenImage
    end
    
    methods (Access = public)
        function startRecording(obj, filePath, filterName)
            if exist(filePath, 'file') == 2
                delete(filePath);
            end
            %obj.m_MatFile = matfile(filePath, 'Writable', true);
            %obj.m_MatFile.FilterName = filterName;
            [dirPath, name, ~] = fileparts(filePath);
            obj.m_FramesFolder = fullfile(dirPath, sprintf('%s_frames', name));
            if ~exist(obj.m_FramesFolder, 'dir')
                mkdir(obj.m_FramesFolder);
            end
            
            % --- TWORZENIE UKRYTEJ FIGURY DO ZAPISU PNG ---
            % Position: [x, y, szerokość, wysokość] - wymusza szerszy format
            obj.m_HiddenFig = figure('Visible', 'off', 'Position', [0, 0, 1200, 600], 'Color', 'black');
            obj.m_HiddenAx = axes('Parent', obj.m_HiddenFig);
            obj.m_HiddenImage = []; % Inicjalizacja uchwytu obrazu
            disableDefaultInteractivity(obj.m_HiddenAx); 
            obj.m_HiddenAx.Toolbar.Visible = 'off';
            obj.m_FrameCount = 0;
            obj.m_IsRecording = true;
        end
        
        function addFrame(obj, cafMatrix, filterParams, rangeAxis, dopplerAxis, sourceFilename)
            if ~obj.m_IsRecording
                return;
            end
            
            obj.m_FrameCount = obj.m_FrameCount + 1;
            fc = obj.m_FrameCount;
            
            % Zapis danych matematycznych do pliku .mat
            % obj.m_MatFile.CafMovie(fc, 1) = {single(cafMatrix)};
            % obj.m_MatFile.FilterParams(fc, 1) = {filterParams};
            % obj.m_MatFile.RangeAxes(fc, 1) = {rangeAxis};
            % obj.m_MatFile.DopplerAxes(fc, 1) = {dopplerAxis};
            % obj.m_MatFile.SourceFiles(fc, 1) = {sourceFilename};
            if isempty(obj.m_HiddenAx) || ~isvalid(obj.m_HiddenAx)
                return;
            end

            % --- RENDEROWANIE I ZAPIS UKRYTEJ KLATKI ---
            % Aktualizacja wykresu (tworzymy raz, potem podmieniamy dane dla szybkości)
            if isempty(obj.m_HiddenImage) || ~isvalid(obj.m_HiddenImage)
                obj.m_HiddenImage = imagesc(obj.m_HiddenAx, dopplerAxis, rangeAxis, cafMatrix);
                obj.m_HiddenAx.YDir = 'normal';
                axis(obj.m_HiddenAx, 'xy');
                colormap(obj.m_HiddenAx, 'jet');
                colorbar(obj.m_HiddenAx);
                xlabel(obj.m_HiddenAx, 'Doppler [Hz]');
                ylabel(obj.m_HiddenAx, 'Range [km]');
            else
                obj.m_HiddenImage.CData = cafMatrix;
                obj.m_HiddenImage.XData = dopplerAxis;
                obj.m_HiddenImage.YData = rangeAxis;
                drawnow limitrate
            end
            
            % Obliczanie limitów kolorów dokładnie jak w playCafMovie
            maxPeak = max(cafMatrix, [], 'all');
            meanPeax = mean(cafMatrix, 'all');
            
            if ~isnan(maxPeak) && ~isempty(maxPeak) && ~isnan(meanPeax) && ~isempty(meanPeax)
                clim(obj.m_HiddenAx, [meanPeax, maxPeak]);
            end
            
            % Dodawanie napisu na górze (zabezpieczenie na typ sourceFilename)
            if isstruct(sourceFilename) && isfield(sourceFilename, 'fileIdx')
                titleStr = sprintf('Klatka: %d\nPlik nr: %d | Nazwa: %s', fc, sourceFilename.fileIdx, sourceFilename.filename);
            else
                titleStr = sprintf('Klatka: %d\nPlik: %s', fc, string(sourceFilename));
            end
            title(obj.m_HiddenAx, titleStr, 'Interpreter', 'none');
            
            % Eksport zrzuconego widoku do pliku PNG
            fileName = sprintf('frame_%05d.png', fc);
            fullImgPath = fullfile(obj.m_FramesFolder, fileName);
            
            % exportgraphics jest polecany w nowym MATLABie zamiast saveas - daje ostre, idealne grafiki
            exportgraphics(obj.m_HiddenFig, fullImgPath, 'Resolution', 120);
        end
        
        function stopRecording(obj)
            obj.m_IsRecording = false;
            %obj.m_MatFile = [];
            
            % Zamknięcie ukrytej figury, aby zwolnić pamięć
            if ~isempty(obj.m_HiddenFig) && isvalid(obj.m_HiddenFig)
                close(obj.m_HiddenFig);
            end
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
                colormap(ax,'jet'); 
                colorbar(ax);
                
                maxPeak = max(cafFrame, [], 'all');
                meanPeax = mean(cafFrame,"all");
                
                if isnan(maxPeak) || isempty(maxPeak) || isnan(meanPeax) || isempty(meanPeax)
                    return;
                end
                clim(ax, [meanPeax, maxPeak]);
                
                xlabel(ax, 'Doppler [Hz]');
                ylabel(ax, 'Range [km]');
                title(ax, sprintf('Klatka: %d / %d \n Plik nr: %d \n Nazwa pliku: %s\n', i, numFrames, sourceFile.fileIdx, sourceFile.filename), 'Interpreter', 'none');
                drawnow;
            end
        end
        
      function createVideoFromPngFolder(framesFolder, outputFilename, fps)
            filePattern = fullfile(framesFolder, '*.png');
            imageFiles = dir(filePattern);
            [~, idx] = sort([imageFiles.datenum]);
            imageFiles = imageFiles(idx);
            numFrames = length(imageFiles);
            
            if numFrames == 0
                error('VideoCreator:NoFiles', 'Nie znaleziono plików PNG we wskazanym folderze.');
            end
            
            firstImgPath = fullfile(framesFolder, imageFiles(1).name);
            firstImg = imread(firstImgPath);
            [targetHeight, targetWidth, ~] = size(firstImg);
            
            v = VideoWriter(outputFilename, "Motion JPEG AVI");
            v.FrameRate = fps;
            v.Quality = 95; 
            open(v);
            
            for i = 1:numFrames
                imgPath = fullfile(framesFolder, imageFiles(i).name);
                img = imread(imgPath);
                
                [h, w, ~] = size(img);
                if h ~= targetHeight || w ~= targetWidth
                    img = imresize(img, [targetHeight, targetWidth]);
                end
                
                writeVideo(v, img);
                if mod(i,100) == 0
                    fprintf('Przetworzono %d / %d klatek...\n', i, numFrames);
            
                end
            end
            close(v);
            fprintf('Zapis wideo zakończony: %s\n', outputFilename);
        end
    end
end
properties (Access = private)
    Model        RadarModel
    Provider     BaseDataProvider
    LoopTimer    timer
end

methods (Access = private)
    function startupFcn(app)
        % Inicjalizacja architektury
        app.Model = RadarModel();
        
        % Subskrypcja zdarzenia z Modelu (Wzorzec Obserwatora)
        addlistener(app.Model, 'DataProcessed', @app.updateUI);
    end
    
    function updateUI(app, ~, ~)
        % Ta funkcja uruchomi się automatycznie po zakończeniu liczenia DSP
        % Rysowanie głównej mapy CAF
        imagesc(app.UIAxes_CAF, app.Model.LastCAFMap);
        
        % Aktualizacja opcjonalnych paneli, jeśli są widoczne
        if app.SpectrumPanel.Visible
            plot(app.UIAxes_Spec, app.Model.LastSpectrum);
        end
    end
    
    % Reakcja na zmianę trybu w UI (Wzorzec Strategii)
    function ModeDropDownValueChanged(app, event)
        switch app.ModeDropDown.Value
            case 'Nagranie (Plik)'
                app.Provider = FileDataProvider({'file1.mat', 'file2.mat'});
            case 'Real-time (C++)'
                app.Provider = CppBufferDataProvider();
        end
    end
    
    % Uruchomienie przetwarzania (np. kliknięcie START)
    function StartButtonPushed(app, event)
        % Dla trybu Real-Time lub Odtwarzania konfigurujemy pętlę Timer
        app.LoopTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.05, ...
            'TimerFcn', @(~,~) app.processingStep);
        start(app.LoopTimer);
    end
    
    function processingStep(app)
        [ref, surv, success] = app.Provider.getNextChunk();
        if success
            % Przekaż parametry z suwaków/pól GUI do modelu
            app.Model.FilterParams.Length = app.FilterLengthSpinner.Value;
            
            % Uruchom processing
            app.Model.processSignals(ref, surv);
        else
            stop(app.LoopTimer);
        end
    end
end
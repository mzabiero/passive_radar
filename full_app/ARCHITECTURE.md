# Architektura systemu Radaru Pasywnego (MATLAB OOP)

## 1. Filozofia i wzorce projektowe

| Wzorzec | Gdzie | Po co |
|---|---|---|
| **Strategy** | `BaseDataProvider` (File/Cpp/Simulation), `IClutterFilter` (ECA/NLMS/RLS), `ICAFAlgorithm` (Direct/BatchesFFT) | Podmiana algorytmu/źródła danych bez zmiany kodu klienta |
| **Factory Method** | `FileParserFactory`, `ClutterFilterFactory`, `CAFAlgorithmFactory` | Jeden punkt tworzenia obiektów ze string-identyfikatora (idealne pod dropdown w App Designer) |
| **Adapter** | `BaseFileParser` + implementacje | Ujednolicenie różnych układów pamięci plików nagrań pod jeden kontrakt `readChunk` |
| **Pipeline / Chain of Responsibility** | `ProcessingPipeline` + `IProcessingStage` | Dynamicznie modyfikowalna sekwencja kroków DSP — **kluczowe** dla wymagania "dołóż CLEAN i powiel je wielokrotnie" |
| **Observer** | eventy `RadarEngine` (`NewFrameProcessed`, ...) + `addlistener` w App | UI aktualizuje się asynchronicznie względem logiki, panele nie znają się nawzajem |
| **DTO / Value object** | `ProcessingContext`, `RadarEngineConfig` | Jawny, jednoznaczny kontrakt danych przepływających przez pipeline |
| **Repository / Memento** | `CAFRecorder` + `CAFFrameIndex` | Zapis wyników jako "nagranie" + swobodny (random-access) odczyt do playbacku |

Zasada nadrzędna: **RadarEngine nigdy nie wie, SKĄD pochodzą dane** (to rola Providera) **ani JAK dokładnie liczony jest CAF/clutter removal** (to rola Strategy schowanych za Stage'ami). Dzięki temu można dowolnie mieszać: plik + ECA + BatchesFFT, symulacja + NLMS + Direct, itd.

## 2. Struktura katalogów

```
PassiveRadarApp/
├── PassiveRadarApp.mlapp          (tworzone w App Designer - patrz sekcja 6)
├── ARCHITECTURE.md
└── +radar/
    ├── +core/
    │   ├── RadarEngine.m            orkiestrator (handle)
    │   ├── ProcessingPipeline.m     dynamiczna lista etapow
    │   ├── IProcessingStage.m       interfejs etapu (abstract)
    │   ├── ProcessingContext.m      DTO przekazywany przez pipeline
    │   ├── RadarEngineConfig.m      value class - konfiguracja
    │   └── ErrorEventData.m         event.EventData dla ProcessingError
    ├── +providers/                  Strategy: zrodla danych
    │   ├── BaseDataProvider.m
    │   ├── FileDataProvider.m       tryb "Analiza nagran"
    │   ├── CppBufferDataProvider.m  tryb "Real-Time" (USRP B210 + C++ SHM)
    │   ├── SimulationDataProvider.m tryb "Symulacja"
    │   └── CAFPlaybackProvider.m    playback zapisanych wynikow
    ├── +parsers/                    Adapter/Factory: formaty plikow
    │   ├── BaseFileParser.m
    │   ├── FileParserFactory.m
    │   └── RawInterleavedParser.m   (przyklad; analogicznie UsrpFileParser, MatFileParser)
    ├── +filters/                    Strategy: usuwanie clutteru
    │   ├── IClutterFilter.m
    │   ├── ClutterFilterFactory.m
    │   └── EcaFilter.m              (przyklad; analogicznie NlmsFilter, RlsFilter)
    ├── +caf/                        Strategy: algorytmy CAF
    │   ├── ICAFAlgorithm.m
    │   ├── CAFAlgorithmFactory.m
    │   └── DirectCorrelationCAF.m   (przyklad; analogicznie BatchesFFTCAF)
    ├── +stages/                     Adaptery Strategy -> IProcessingStage
    │   ├── PreProcessingStage.m
    │   ├── ClutterRemovalStage.m
    │   ├── CAFComputationStage.m
    │   └── CleanStage.m             etap powielalny wielokrotnie
    ├── +io/                         zapis / indeksacja wynikow
    │   ├── CAFRecorder.m
    │   └── CAFFrameIndex.m
    ├── +simulation/
    │   ├── ScenarioManager.m
    │   └── SimulatedTarget.m
    └── +reprocessing/
        └── SegmentReprocessingSession.m   reanaliza wybranego fragmentu
```

Uwaga: pakiety `+parsers`, `+filters`, `+caf` zawierają po jednej przykładowej implementacji — kolejne (`NlmsFilter`, `RlsFilter`, `BatchesFFTCAF`, `UsrpFileParser`, `MatFileParser`) piszesz analogicznie, implementując tylko właściwy interfejs i dopisując `case` w odpowiedniej fabryce. GUI (dropdown) automatycznie pokaże nową opcję, bo czerpie listę z `XxxFactory.availableXxx()`.

## 3. Główna pętla zdarzeń (tryb "live": plik lub real-time)

```
timer.TimerFcn (co np. 100 ms):
  1. [ref, surv, meta, ok] = app.Provider.getNextChunk()
  2. if ~ok:
       if strcmp(app.Mode,"file") -> koniec pliku, zatrzymaj timer, pokaz podsumowanie
       else -> brak nowych danych w SHM w tej chwili, po prostu return (real-time)
  3. app.Engine.processFrame(ref, surv, meta)
     -> wewnatrz: Pipeline.execute(context) + notify('NewFrameProcessed')
  4. (asynchronicznie, w listenerze zarejestrowanym w startupFcn):
       panelCAF.updatePlot(app.Engine.LastContext)
       panelSpectrum.updatePlot(app.Engine.LastContext)
```

Rejestracja listenera w `startupFcn` Aplikacji (View jako kontroler):

```matlab
function startupFcn(app)
    app.Engine = radar.core.RadarEngine(app.buildConfigFromUI());
    addlistener(app.Engine, 'NewFrameProcessed', @(~,~) app.onNewFrame());
    addlistener(app.Engine, 'ProcessingError',   @(~,evt) app.onEngineError(evt));
end

function onNewFrame(app)
    ctx = app.Engine.LastContext;
    % Update "cheap": tylko podmiana ZData istniejacego wykresu,
    % NIE tworzenie nowego surf/imagesc za kazdym razem.
    set(app.CAFSurface, 'CData', ctx.CAFMatrix);
    drawnow limitrate
end
```

## 4. Podmiana algorytmów "w locie" z poziomu GUI

Dropdown "Filtr clutteru" (`ECA` / `NLMS` / `RLS`) wypełniany z `radar.filters.ClutterFilterFactory.availableFilters()`. Callback:

```matlab
function ClutterFilterDropDownValueChanged(app, ~)
    app.Engine.setClutterFilter(app.ClutterFilterDropDown.Value, app.currentFilterParams());
end
```

`RadarEngine.setClutterFilter` odnajduje `ClutterRemovalStage` w pipeline przez `findStage("ClutterRemoval")` i podmienia mu wyłącznie wewnętrzny obiekt `IClutterFilter` (z wywołaniem `reset()`, żeby uniknąć niespójnego stanu np. w filtrach adaptacyjnych NLMS/RLS). **Pozycja etapu w pipeline i wszystkie bindingi UI pozostają nienaruszone** — podmieniana jest tylko strategia wewnątrz stałego "gniazda". Analogicznie działa `setCAFAlgorithm`.

Ponieważ MATLAB w tym kontekście jest jednowątkowy (timer wykonuje się między klatkami), podmiana nigdy nie nastąpi "w połowie" liczenia ramki — `RadarEngine.IsProcessing` dodatkowo zabezpiecza przed reentrancy, gdyby ktoś w przyszłości dołożył `parfeval` do CAF.

## 5. Nagrywanie i odtwarzanie wyników (kluczowe wymaganie)

**Podczas analizy pliku:**

```matlab
app.Engine.attachRecorder(radar.io.CAFRecorder(outputPathFromUI));
app.Recorder.start();
% ... petla timera jak wyzej, RadarEngine sam woła Recorder.writeFrame() ...
% po zakonczeniu (DataExhausted):
app.Recorder.stop();   % zapisuje CAFFrameIndex (.idx.mat) obok danych (.caf)
```

Każdy wpis w `CAFFrameIndex` przechowuje nie tylko `ByteOffset`/`ByteLength` (do odczytu klatki CAF), ale też `SourceFile` + `SourceSampleOffset` — **wskaźnik powrotny do surowego IQ**, z którego ta klatka powstała. To jest most do reprocessingu.

**Playback:** `CAFPlaybackProvider` ładuje `.idx.mat` (mały plik) i czyta konkretne klatki na żądanie (`getFrame`/`getRange`) — nigdy nie wczytuje całego wyniku do RAM, identycznie jak `FileDataProvider` przy pierwotnym przetwarzaniu.

## 6. Reprocessing wybranego fragmentu (scenariusz "dron 2–10 min")

```matlab
% 1. Uzytkownik zaznacza zakres na osi czasu playbacku:
range = [minutes(2), minutes(10)];

% 2. Tworzymy sesje - Provider jest OD RAZU zawezony do wlasciwych
%    probek zrodlowych (znalezionych przez CAFFrameIndex.findRange):
session = radar.reprocessing.SegmentReprocessingSession( ...
    app.PlaybackProvider.Index, range, app.SourceFolder, app.Engine.Config);

% 3. Edytor pipeline'u w GUI modyfikuje session.Engine.Pipeline:
session.customizePipeline(@(p) p.addStage(radar.stages.CleanStage()));
idx = session.Engine.Pipeline.findStageIndex("CLEAN");
session.customizePipeline(@(p) p.duplicateStage(idx));  % 2x CLEAN
session.customizePipeline(@(p) p.duplicateStage(idx));  % 3x CLEAN

% 4. Uruchomienie (petla offline, szybciej niz real-time):
addlistener(session, 'SegmentProgress', @(~,~) app.updateProgressBar());
session.run();

% 5. Wynik - osobny, nie nadpisujacy oryginalu zestaw plikow:
session.exportResults(fullfile(app.AnalysisFolder, "drone_segment_3xCLEAN"));
```

Ta sama konstrukcja pozwala trzymać **kilka równoległych sesji** dla tego samego fragmentu (np. do porównania "1× CLEAN" vs "3× CLEAN" vs "CLEAN + inny filtr clutteru") — każda to niezależny `RadarEngine` + `ProcessingPipeline`, nic nie jest współdzielone poza źródłowym plikiem (odczyt read-only).

## 7. Wydajność i wątkowość

- **Sekwencyjny odczyt bez przepełnienia RAM**: `FileDataProvider`/`BaseFileParser` trzymają uchwyt pliku (`fopen`) i czytają kolejne bloki `fread`; nigdy `load()` całego pliku. Dla `.mat` v7.3 analogicznie użyj `matfile()` z indeksowaniem wycinków zamiast pełnego `load`.
- **UI aktualizowane "tanio"**: w handlerze `NewFrameProcessed` podmieniaj `CData`/`ZData` istniejącego obiektu graficznego (`set(...)`), nie twórz wykresu od nowa. Rozważ przerysowywanie co N-tą klatkę w trybie "fast forward" (przetwarzanie szybsze niż realtime), niezależnie od częstotliwości liczenia.
- **Timer**: `ExecutionMode='fixedSpacing'`, `BusyMode='drop'` — jeśli przetwarzanie jednej ramki trwa dłużej niż okres timera, kolejne wywołanie jest pomijane zamiast się kolejkować.
- **Reentrancy guard**: `RadarEngine.IsProcessing` chroni przed nałożeniem się dwóch wywołań `processFrame` (istotne, gdyby GUI pozwalało zmieniać strategię w trakcie przetwarzania klatki).
- **CAF jako wąskie gardło**: jeśli obliczenia będą zbyt wolne dla realtime, rozważ przeniesienie `ICAFAlgorithm.computeCAF` na `parfeval` z Parallel Computing Toolbox — `CAFComputationStage` pozostaje wtedy synchronicznym API, a asynchroniczność chowa się wewnątrz konkretnej implementacji algorytmu (`Future` + odbiór wyniku w kolejnym "ticku" timera). Interfejs `ICAFAlgorithm` nie musi się zmieniać.
- **CleanStage jako koszt świadomy**: celowo NIE wchodzi w skład domyślnego pipeline'u trybu live — jego iteracyjny, potencjalnie wielokrotny charakter (patrz sekcja 6) czyni go narzędziem do dokładnej, offline'owej reanalizy, a nie do przetwarzania w czasie rzeczywistym.

## 8. Rozszerzanie systemu — checklist

| Chcę dodać... | Kroki |
|---|---|
| Nowy filtr clutteru (np. ECA-B) | 1) nowa klasa `radar.filters.EcaBFilter < IClutterFilter`  2) `case "ECA-B"` w `ClutterFilterFactory`  3) dopisz do `availableFilters()` |
| Nowy algorytm CAF | analogicznie w `radar.caf` / `CAFAlgorithmFactory` |
| Nowy format pliku nagrania | nowa klasa `< BaseFileParser`, wpis w `FileParserFactory.Registry` |
| Nowy etap pipeline'u (np. CFAR) | nowa klasa `< IProcessingStage`, dodawana przez `Pipeline.addStage()` — nie wymaga zmian w `RadarEngine` |
| Nowe źródło real-time (np. RTL-SDR) | nowa klasa `< BaseDataProvider` — zero zmian w `RadarEngine`/UI poza wyborem providera w `startupFcn` |

## 9. Szkielet `PassiveRadarApp.mlapp` (App Designer)

Plik `.mlapp` jest binarny i tworzy się go w edytorze App Designer, ale poniżej jest struktura właściwości/callbacków, którą warto tam odwzorować:

```matlab
properties (Access = private)
    Engine            radar.core.RadarEngine
    Provider          % BaseDataProvider (polimorficznie: File/Cpp/Simulation)
    Recorder          radar.io.CAFRecorder
    PlaybackProvider  radar.providers.CAFPlaybackProvider
    ActiveSession     radar.reprocessing.SegmentReprocessingSession
    MainTimer         timer
end

methods (Access = private)
    function RunButtonPushed(app, ~)
        app.Provider = app.createProviderFromUISelection(); % Strategy - wybor wg trybu
        app.Provider.open();
        app.Engine = radar.core.RadarEngine(app.buildConfigFromUI());
        addlistener(app.Engine, 'NewFrameProcessed', @(~,~) app.onNewFrame());
        if app.SaveResultsCheckBox.Value
            app.Recorder = radar.io.CAFRecorder(app.OutputPathField.Value);
            app.Recorder.start();
            app.Engine.attachRecorder(app.Recorder);
        end
        app.MainTimer = timer('ExecutionMode','fixedSpacing', 'Period', 0.1, ...
            'BusyMode','drop', 'TimerFcn', @(~,~) app.onTimerTick());
        start(app.MainTimer);
    end

    function onTimerTick(app)
        [ref, surv, meta, ok] = app.Provider.getNextChunk();
        if ~ok
            stop(app.MainTimer);
            if ~isempty(app.Recorder), app.Recorder.stop(); end
            return
        end
        app.Engine.processFrame(ref, surv, meta);
    end

    function TimeRangeSelected(app, startT, endT)
        % wywolywane przez widget zaznaczania zakresu w panelu playbacku
        app.ActiveSession = radar.reprocessing.SegmentReprocessingSession( ...
            app.PlaybackProvider.Index, [startT endT], app.SourceFolderField.Value, ...
            app.Engine.Config);
        app.openPipelineEditor(app.ActiveSession.Engine.Pipeline);
    end
end
```

Panele UI (spektrum, mapa CAF, edytor pipeline'u, oś czasu playbacku) implementuj jako **osobne, nie-wizualne klasy kontrolerów** (`radar.ui.CAFMapPanelController` itp.), które w konstruktorze dostają referencję do `UIAxes`/`UITable` z `.mlapp` oraz subskrybują odpowiednie eventy `Engine`/`Session` — to utrzymuje `.mlapp` chudy (tylko layout + delegacja), zgodnie z zasadą, że `.mlapp` pełni rolę kontrolera, a nie logiki.

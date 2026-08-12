function plot_spectrum(x, fs, fig)
% WYSWIETL_WIDMO - wyświetla widmo amplitudowe sygnału zespolonego
%
%   x     - sygnał (wektor zespolony)
%   fs    - częstotliwość próbkowania [Hz]
%   tytul - tytuł wykresu (string)
    if nargin < 3
        fig = figure;
    end
    N = length(x);                      % liczba próbek
    X = fftshift(fft(x));              % przesunięta FFT
    f = fs * (-N/2:N/2-1) / N;         % oś częstotliwości  
    X_norm = X / max(abs(X));
    X_dB = mag2db(abs(X_norm));


    plot(f, X_dB);
    %xlabel(fig,'Częstotliwość [Hz]');
    %ylabel(fig,'|X(f)|');
    %grid(fig,"on");
end

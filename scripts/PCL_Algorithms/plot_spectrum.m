function plot_spectrum(x, fs)
% WYSWIETL_WIDMO - wyświetla widmo amplitudowe sygnału zespolonego
%
%   x     - sygnał (wektor zespolony)
%   fs    - częstotliwość próbkowania [Hz]
%   tytul - tytuł wykresu (string)

    N = length(x);                      % liczba próbek
    X = fftshift(fft(x));              % przesunięta FFT
    f = fs * (-N/2:N/2-1) / N;         % oś częstotliwości  
    X_norm = X / max(abs(X));
    X_dB = mag2db(abs(X_norm));

    figure;
    plot(f, X_dB);
    xlabel('Częstotliwość [Hz]');
    ylabel('|X(f)|');
    grid on;
end

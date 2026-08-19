function ax = plotSpectrum(sig, fs, fc, ax, ttl)
    if nargin < 4 || isempty(ax) || ~isvalid(ax)
        fig = figure('Name', 'Frequency Spectrum', 'NumberTitle', 'off');
        ax = axes(fig);
    end
    cla(ax);
    N = length(sig);
    spectrum = fftshift(fft(sig));
    spectrumMag = 10 * log10(abs(spectrum) + eps);
    freqAxis = linspace(-fs/2, fs/2, N) + fc;
    
    plot(ax, freqAxis, spectrumMag);
    grid(ax, 'on');
    
    xlabel(ax, 'Częstotliwość [Hz]');
    ylabel(ax, 'Amplituda [dB]');
    if nargin < 5
        title(ax, 'Widmo częstotliwościowe');
    else
        title(ax, ttl);
    end
end
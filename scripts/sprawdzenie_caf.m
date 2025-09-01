clear all
close all
clc

fs = 200e3;
T = 1; 

wn = create_wn(fs,T, "complex");


% % Projekt filtru FIR dolnoprzepustowego
fpass = 50e3;                  % przepustowość filtru [Hz]
Wn = fpass / (fs/2);           % znormalizowana częstotliwość odcięcia
b = fir1(256, Wn);             % filtr FIR (256-rzędowy)

% Filtracja szumu
filtered_noise_ref = filter(b, 1, wn);



CAF(filtered_noise_ref,filtered_noise_ref,fs,1000,500,148,50);
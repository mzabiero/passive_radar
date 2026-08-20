clear all; close all; clc

T = 0.5;
fs = 20e6;
fc = 500e6;
N = fs * T;
f = 10;
t = (0:(N-1)).' / fs;
c = 3e8;
lambda = c / fc;

tgt_r = 500;
tgt_v = 80;
tgt_del_t = tgt_r / c;
tgt_del_samp = round(tgt_del_t * fs);
tgt_fd = - tgt_v / lambda;

x_ref = randn(N,1) + 1i* randn(N,1);
x_surv = zeros(N,1);

idx = tgt_del_samp:N;
%x_surv(idx) = x_ref(1:end-tgt_del_samp+1); %.* exp(1j * 2 * pi * tgt_fd * t(idx));
%x_surv_comp = x_surv .* exp(1j * 2 * pi * tgt_fd * t);
Gui.plotSpectrum(x_ref, fs, fc);
%x_surv = x_surv * exp(1j * 2 * pi / lambda * tgt_v * t);
%[x_cor, lags] = xcorr(x_surv, x_ref, 200);
%plot(lags, abs(x_cor));
%plot(t,abs(x));
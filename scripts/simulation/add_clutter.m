function x_surv = add_clutter(x_surv, fs, clutterParams)
% Add clutter (stationary & spread)
% clutterParams: struct with fields
%   .power  (relative power wrt signal)
%   .spread (Hz Doppler spread, e.g. [0.5, 1] Hz for slow movers)
%   .num    (number of clutter echoes)
    N = length(x_surv);
    n = (0:N-1).';
    
    for k = 1:clutterParams.num
        dopplerHz = rand()*diff(clutterParams.spread) + clutterParams.spread(1);
        phase = exp(1j*2*pi*dopplerHz*n/fs);
        clutter = sqrt(clutterParams.power/clutterParams.num) * phase .* (0.5 - rand(N,1));
        x_surv = x_surv + clutter;
    end
end

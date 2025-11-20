function clutter = create_clutter(range_m, mag_dB)
    if(numel(range_m) ~= numel(mag_dB))
        error("Delay and magnitude of clutter have to be the same size");
    end
    N = numel(range_m);

    % Preallocate struct array (N×1 column)
    clutter = repmat(struct('range_m',0,'magnitude',0), N, 1);
    
    for i = 1:numel(range_m)
        clutter(i).range_m = range_m(i);
        clutter(i).magnitude = mag_dB(i);
    end
end
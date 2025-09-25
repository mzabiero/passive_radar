function save_complex_binary(sig, fname)
% saveIQ - save complex signal as raw binary
% sig   : complex baseband signal (vector)
% fname : output filename (.dat)

    % open file
    fid = fopen(fname, 'wb');
    if fid < 0
        error('Could not open file %s for writing.', fname);
    end
    
    % interleave real and imag parts as float32 (like gnuradio .dat files)
    x = [real(sig(:)).'; imag(sig(:)).'];
    fwrite(fid, x, 'float32');
    
    fclose(fid);
end
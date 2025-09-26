function save_complex_binary(sig, fname)
% save_complex_binary(sig,fname) - save complex signal as raw binary
% (interleaved)
% sig   : complex baseband signal (vector)
% fname : output filename (.dat)

    
    fid = fopen(fname, 'wb'); % open file in append mode
    if fid < 0
        error('Could not open file %s for writing.', fname);
    end
    
    % interleave real and imag parts as float32 (like gnuradio .dat files)
    x = [real(sig(:)).'; imag(sig(:)).'];
    fwrite(fid, x, 'float32');
    
    fclose(fid);
end
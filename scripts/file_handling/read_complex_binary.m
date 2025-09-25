function signal = read_complex_binary(filename)
    fid = fopen(filename, 'rb');
    raw = fread(fid, 'float32');
    fclose(fid);
    signal = raw(1:2:end) + 1i * raw(2:2:end);

end
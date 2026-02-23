function split_signal(filename, duration,fs)
    fid = fopen(filename);
    data = fread(fid, 'float32'); % Read the signal data from the file
    fclose(fid); % Close the file after reading
    
    N = size(data);
    sig1 = data(1:duration);
    

end
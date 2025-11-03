function window_ym = window_CAF(len,type) 
    
    
        fprintf("Creating window: %s\n", type);
        window_size = ceil(len);

        switch lower(type)
            case 'hamming'
                window_ym =  hamming(window_size);
            case 'hann'
                window_ym =  hann(window_size);
            case 'blackmann'
                window_ym =  blackmanharris(window_size);
            case 'kaiser'
                beta = 5;
                window_ym =  kaiser(window_size, beta);
            otherwise
                window_ym = ones(ceil(len),1);
        end
    end


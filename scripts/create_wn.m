function white_noise = create_wn(fs, T, real_complex)

    N = fs*T;    
    if real_complex == "complex"
        white_noise = randn(N,1) + 1i*randn(N,1);
    elseif real_complex == "real"
        white_noise = randn(N,1);
    end
  

end
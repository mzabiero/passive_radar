function [range_km, velocity_ms] = bins2kmms(delay_idx, doppler_idx,fs, doppler_bins)
    c =3e8;
    bistatic_range_km = (c * (delay_idx / fs)) / 1000;
    
end
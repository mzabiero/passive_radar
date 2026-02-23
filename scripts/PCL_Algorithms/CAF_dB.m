function CAF_dB = CAF_dB(CAF)
    CAF_dB = mag2db(abs(CAF) + eps);
end
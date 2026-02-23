function pCLEAN(x_ref,x_surv,params,ax)
    fs = params.fs;
    fc = params.fc;
    max_delay = params.max_delay;
    doppler_bins = params.doppler_bins;

    [caf_mag, d_ax, v_ax] =CAF(x_ref,x_surv,fs,fc,max_delay,doppler_bins,params.R,params.window_type,0);
    caf_db = CAF_dB(caf_mag);
    plotCAF(caf_mag,d_ax,v_ax,"CLEAN",ax);
    %ax.CLim = [mean(caf_db(:)), 0];
    ax.CLim = [mean(caf_db(:)), max(caf_db(:))];
end
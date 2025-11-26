function x_surv_clean = clutter_removal( ...
        x_ref, x_surv, filt_order, forgetting_fact, block_len, back_filt)

    disp("Clutter removal running...");
    
    N = length(x_ref);
    M = block_len;
    nBlocks = floor(N/M);
    if back_filt > 0
        ref_shifted = [x_ref(back_filt+1:end); zeros(back_filt,1)];
    else
        ref_shifted = x_ref;
    end

    x_surv_clean = complex(zeros(N,1));

    lattice = dsp.AdaptiveLatticeFilter(...
        "Method","Least-squares Lattice",...
        "ForgettingFactor",forgetting_fact,...
        "Length",filt_order);

    idx = 1;

    for k = 1:nBlocks

        r_block = ref_shifted(idx : idx+M-1);
        s_block = x_surv(idx : idx+M-1);

        [~, e_block] = lattice(r_block, s_block);

        x_surv_clean(idx:idx+M-1) = e_block;

        idx = idx + M;
    end

end

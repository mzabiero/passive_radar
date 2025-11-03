function x_surv_clean = clutter_removal (x_ref, x_surv,filt_order,...
    step_size, block_len)

    disp("Clutter removal running...");
    %parameters
    N = length(x_ref);
    M = block_len;
    nBlocks = floor(N/M);
    x_surv_clean = complex(zeros(N,1));

    %filter design 
    lattice = dsp.AdaptiveLatticeFilter(...
        "Length", filt_order,...
        "StepSize", step_size,...
        "Method","Gradient Adaptive Lattice");

    
    idx = 1;

    for k = 1:nBlocks

        ref_block = x_ref(idx:idx+M-1);
        surv_block = x_surv(idx:idx+M-1);

        [y_block, e_block] = lattice(ref_block,surv_block);

        x_surv_clean(idx:idx+M-1) = e_block;

        idx = idx+M;
    end
    % [y,e] = lattice(x_ref,x_surv);
    %x_surv_clean = e; 
end
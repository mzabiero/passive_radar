function [max_val, delay_idx, doppler_idx] = find_max_CLEAN(caf_matrix)
    [max_val, linear_idx] = max(caf_matrix(:));
    [delay_idx, doppler_idx] = ind2sub(size(caf_matrix), linear_idx);
end
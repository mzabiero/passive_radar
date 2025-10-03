function x_surv = add_dpi(dpi_atten,x_ref,x_surv)
%ADD DIRECT PATH INTERFERANCE
    x_surv = x_surv + dpi_atten * x_ref;
end 
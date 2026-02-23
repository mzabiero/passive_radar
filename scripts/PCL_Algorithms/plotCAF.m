function plotCAF(caf, delay_axis, doppler_axis, ttl,ax)
    caf = CAF_dB(caf);
    if isempty(delay_axis)
        delay_axis = 1:size(caf,1);
        doppler_axis = 1:size(caf,2);
    end
    imagesc(ax, doppler_axis, delay_axis, caf); 
    axis(ax,'xy');
    colormap(ax,'jet'); colorbar(ax);
    title(ax, ttl,'FontSize',12);
     
end
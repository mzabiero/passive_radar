function sim_save_del(infile,x_ref,x_surv)

    [pathstr,name] = fileparts(infile);
    outRef = fullfile(pathstr, [name '_ref.dat']);
    outSurv = fullfile(pathstr, [name '_surv.dat']);

    save_comlex_binary(x_ref,outRef);
    save_comlex_binary(x_surv,outSurv);

    if isfile(infile)
        delete(infile);
    end

    fprintf('Saved:\n  %s\n  %s\n', outRef, outSurv);
end